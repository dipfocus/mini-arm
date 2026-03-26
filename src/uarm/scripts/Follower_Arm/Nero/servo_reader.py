import argparse
import logging
import re
import sys
import threading
import time
from pathlib import Path

import serial

SRC_DIR = Path(__file__).resolve().parents[4]
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from uarm.scripts.Follower_Arm.Nero.log_utils import configure_logging


logger = logging.getLogger(__name__)


class ServoReader:
    RESPONSE_TERMINATOR = b"!"
    READ_MODE_SINGLE = "single"
    READ_MODE_BATCH = "batch"
    READ_MODE_AUTO = "auto"

    def __init__(
        self,
        port="/dev/ttyUSB0",
        baudrate=115200,
        servo_count=8,
        first_servo_id=1,
        response_timeout=0.01,
        read_mode=READ_MODE_AUTO,
    ):
        self.SERIAL_PORT = port
        self.BAUDRATE = baudrate
        self.servo_count = int(servo_count)
        self.first_servo_id = int(first_servo_id)
        self.response_timeout = float(response_timeout)
        self.read_mode = str(read_mode).strip().lower()
        if self.servo_count <= 0:
            raise ValueError(f"servo_count must be greater than 0, got {self.servo_count}")
        if self.first_servo_id <= 0:
            raise ValueError(
                f"first_servo_id must be greater than 0, got {self.first_servo_id}"
            )
        if self.response_timeout <= 0.0:
            raise ValueError(
                f"response_timeout must be greater than 0, got {self.response_timeout}"
            )
        if self.read_mode not in {
            self.READ_MODE_SINGLE,
            self.READ_MODE_BATCH,
            self.READ_MODE_AUTO,
        }:
            raise ValueError(
                "read_mode must be one of "
                f"{self.READ_MODE_SINGLE!r}, {self.READ_MODE_BATCH!r}, {self.READ_MODE_AUTO!r}, "
                f"got {read_mode!r}"
            )
        self.ser = serial.Serial(
            self.SERIAL_PORT,
            self.BAUDRATE,
            timeout=self.response_timeout,
            write_timeout=self.response_timeout,
        )
        self.servo_ids = tuple(
            range(self.first_servo_id, self.first_servo_id + self.servo_count)
        )
        self._active_read_mode = (
            self.READ_MODE_SINGLE
            if self.read_mode == self.READ_MODE_SINGLE
            else self.READ_MODE_BATCH
        )
        logger.info("Serial port %s opened", self.SERIAL_PORT)

        self.zero_angles = [0.0] * self.servo_count
        self.current_angles = [0.0] * self.servo_count
        self.last_update_monotonic = 0.0
        self.last_cycle_duration = 0.0
        self.lock = threading.Lock()
        self._stop_event = threading.Event()

        self._init_servos()

    def _clear_input_buffer(self):
        if self.ser.in_waiting:
            self.ser.read(self.ser.in_waiting)

    def _read_response(self, expected_responses=1):
        if expected_responses <= 0:
            return b""

        response = bytearray()
        for _ in range(expected_responses):
            chunk = self.ser.read_until(self.RESPONSE_TERMINATOR)
            if not chunk:
                break
            response.extend(chunk)
            if not chunk.endswith(self.RESPONSE_TERMINATOR):
                break
        if self.ser.in_waiting:
            response.extend(self.ser.read(self.ser.in_waiting))
        return bytes(response)

    def send_command(self, cmd, expected_responses=1):
        self._clear_input_buffer()
        self.ser.write(cmd.encode('ascii'))
        self.ser.flush()
        return self._read_response(expected_responses=expected_responses).decode(
            'ascii',
            errors='ignore',
        )

    @staticmethod
    def _pwm_value_to_angle(pwm_val, pwm_min=500, pwm_max=2500, angle_range=270):
        pwm_span = pwm_max - pwm_min
        return (pwm_val - pwm_min) / pwm_span * angle_range

    def pwm_to_angle(self, response_str, pwm_min=500, pwm_max=2500, angle_range=270):
        match = re.search(r'P(\d{4})', response_str)
        if not match:
            return None
        return self._pwm_value_to_angle(
            int(match.group(1)),
            pwm_min=pwm_min,
            pwm_max=pwm_max,
            angle_range=angle_range,
        )

    def extract_pwm_values(self, response_str):
        return [int(value) for value in re.findall(r'P(\d{4})', response_str)]

    def _read_angles_single(self):
        new_angles = [0.0] * self.servo_count
        for index, servo_id in enumerate(self.servo_ids):
            response = self.send_command(f'#{servo_id:03d}PRAD!')
            angle = self.pwm_to_angle(response.strip())
            if angle is not None:
                new_angles[index] = angle - self.zero_angles[index]
        return new_angles

    def _read_angles_batch(self):
        batched_command = "".join(f'#{servo_id:03d}PRAD!' for servo_id in self.servo_ids)
        response = self.send_command(
            batched_command,
            expected_responses=self.servo_count,
        )
        pwm_values = self.extract_pwm_values(response)
        if len(pwm_values) != self.servo_count:
            logger.warning(
                "Batch servo read expected %d PWM values, got %d: %r",
                self.servo_count,
                len(pwm_values),
                response.strip(),
            )
            return None

        return [
            self._pwm_value_to_angle(pwm_val) - self.zero_angles[index]
            for index, pwm_val in enumerate(pwm_values)
        ]

    def _read_angles(self):
        if self._active_read_mode == self.READ_MODE_BATCH:
            angles = self._read_angles_batch()
            if angles is not None:
                return angles
            logger.warning(
                "Batch servo read failed in %s mode, falling back to single-servo polling",
                self.read_mode,
            )
            self._active_read_mode = self.READ_MODE_SINGLE

        return self._read_angles_single()

    def _init_servos(self):
        self.send_command('#000PVER!')
        for index, servo_id in enumerate(self.servo_ids):
            self.send_command("#000PCSK!")
            self.send_command(f'#{servo_id:03d}PULK!')
            response = self.send_command(f'#{servo_id:03d}PRAD!')
            angle = self.pwm_to_angle(response.strip())
            self.zero_angles[index] = angle if angle is not None else 0.0
        logger.info("Servo initial angle calibration completed")

    def read_loop(self, hz=100):
        if hz <= 0:
            raise ValueError(f"hz must be greater than 0, got {hz}")
        dt = 1.0 / hz
        while not self._stop_event.is_set():
            cycle_start = time.monotonic()
            new_angles = self._read_angles()
            cycle_end = time.monotonic()
            cycle_duration = cycle_end - cycle_start
            with self.lock:
                self.current_angles = new_angles
                self.last_update_monotonic = cycle_end
                self.last_cycle_duration = cycle_duration
            logger.info(
                "Servo read completed: mode=%s | angles=%s | cycle_ms=%.1f",
                self._active_read_mode,
                new_angles,
                cycle_duration * 1000.0,
            )
            remaining = dt - (time.monotonic() - cycle_start)
            if remaining > 0.0:
                self._stop_event.wait(remaining)

    def get_angles(self):
        with self.lock:
            return list(self.current_angles)

    def get_stats(self):
        with self.lock:
            cycle_duration = float(self.last_cycle_duration)
            last_update = float(self.last_update_monotonic)
        sample_hz = 0.0 if cycle_duration <= 0.0 else 1.0 / cycle_duration
        sample_age = None if last_update <= 0.0 else time.monotonic() - last_update
        return {
            "sample_hz": sample_hz,
            "sample_age": sample_age,
            "cycle_duration": cycle_duration,
        }

    def stop(self):
        self._stop_event.set()

    def close(self):
        self.stop()
        if self.ser.is_open:
            self.ser.close()
            logger.info("Serial port %s closed", self.SERIAL_PORT)


def main():
    parser = argparse.ArgumentParser(description="Test ServoReader by continuously printing servo angles.")
    parser.add_argument("--port", default="/dev/ttyUSB0", help="Serial port path.")
    parser.add_argument("--baudrate", default=115200, type=int, help="Serial baudrate.")
    parser.add_argument(
        "--servo-count",
        default=8,
        type=int,
        help="Number of servos to read starting from first-servo-id.",
    )
    parser.add_argument(
        "--first-servo-id",
        default=1,
        type=int,
        help="First servo ID in the chain.",
    )
    parser.add_argument(
        "--read-hz",
        default=100.0,
        type=float,
        help="Background servo polling frequency in Hz.",
    )
    parser.add_argument(
        "--print-hz",
        default=5.0,
        type=float,
        help="Console print frequency in Hz.",
    )
    parser.add_argument(
        "--response-timeout",
        default=0.01,
        type=float,
        help="Serial response timeout in seconds.",
    )
    parser.add_argument(
        "--read-mode",
        default=ServoReader.READ_MODE_AUTO,
        choices=[
            ServoReader.READ_MODE_AUTO,
            ServoReader.READ_MODE_SINGLE,
            ServoReader.READ_MODE_BATCH,
        ],
        help="Servo polling mode. 'auto' tries batch read first and falls back to single.",
    )
    args = parser.parse_args()

    if args.read_hz <= 0:
        parser.error("--read-hz must be greater than 0")
    if args.print_hz <= 0:
        parser.error("--print-hz must be greater than 0")
    if args.response_timeout <= 0:
        parser.error("--response-timeout must be greater than 0")

    configure_logging()

    reader = None
    reader_thread = None
    try:
        reader = ServoReader(
            port=args.port,
            baudrate=args.baudrate,
            servo_count=args.servo_count,
            first_servo_id=args.first_servo_id,
            response_timeout=args.response_timeout,
            read_mode=args.read_mode,
        )
        reader_thread = threading.Thread(
            target=reader.read_loop,
            kwargs={"hz": args.read_hz},
            daemon=True,
        )
        reader_thread.start()

        logger.info("Test started, press Ctrl+C to stop.")
        dt = 1.0 / args.print_hz
        while True:
            stats = reader.get_stats()
            logger.info(
                "Current angles: %s | sample_hz=%.1f | sample_age_ms=%s",
                reader.get_angles(),
                stats["sample_hz"],
                "n/a" if stats["sample_age"] is None else f"{stats['sample_age'] * 1000.0:.1f}",
            )
            time.sleep(dt)
    except KeyboardInterrupt:
        logger.info("Test interrupted")
    except serial.SerialException as exc:
        logger.error("Serial communication failed: %s", exc)
        return 1
    finally:
        if reader is not None:
            reader.stop()
        if reader_thread is not None and reader_thread.is_alive():
            reader_thread.join(timeout=1.0)
        if reader is not None:
            reader.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
