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
    def __init__(self, port="/dev/ttyUSB0", baudrate=115200, servo_count=8, first_servo_id=1):
        self.SERIAL_PORT = port
        self.BAUDRATE = baudrate
        self.servo_count = int(servo_count)
        self.first_servo_id = int(first_servo_id)
        if self.servo_count <= 0:
            raise ValueError(f"servo_count must be greater than 0, got {self.servo_count}")
        if self.first_servo_id <= 0:
            raise ValueError(
                f"first_servo_id must be greater than 0, got {self.first_servo_id}"
            )
        self.ser = serial.Serial(self.SERIAL_PORT, self.BAUDRATE, timeout=0.1)
        logger.info("Serial port %s opened", self.SERIAL_PORT)

        self.zero_angles = [0.0] * self.servo_count
        self.current_angles = [0.0] * self.servo_count
        self.lock = threading.Lock()
        self._stop_event = threading.Event()

        self._init_servos()

    def send_command(self, cmd):
        self.ser.write(cmd.encode('ascii'))
        time.sleep(0.008)
        return self.ser.read_all().decode('ascii', errors='ignore')

    def pwm_to_angle(self, response_str, pwm_min=500, pwm_max=2500, angle_range=270):
        match = re.search(r'P(\d{4})', response_str)
        if not match:
            return None
        pwm_val = int(match.group(1))
        pwm_span = pwm_max - pwm_min
        angle = (pwm_val - pwm_min) / pwm_span * angle_range
        return angle

    def _init_servos(self):
        self.send_command('#000PVER!')
        for index, servo_id in enumerate(range(self.first_servo_id, self.first_servo_id + self.servo_count)):
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
            new_angles = [0.0] * self.servo_count
            for index, servo_id in enumerate(range(self.first_servo_id, self.first_servo_id + self.servo_count)):
                response = self.send_command(f'#{servo_id:03d}PRAD!')
                angle = self.pwm_to_angle(response.strip())
                if angle is not None:
                    new_angles[index] = angle - self.zero_angles[index]
            with self.lock:
                self.current_angles = new_angles
            logger.debug("Servo read completed: angles=%s", new_angles)
            self._stop_event.wait(dt)

    def get_angles(self):
        with self.lock:
            return list(self.current_angles)

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
    args = parser.parse_args()

    if args.read_hz <= 0:
        parser.error("--read-hz must be greater than 0")
    if args.print_hz <= 0:
        parser.error("--print-hz must be greater than 0")

    configure_logging()

    reader = None
    reader_thread = None
    try:
        reader = ServoReader(
            port=args.port,
            baudrate=args.baudrate,
            servo_count=args.servo_count,
            first_servo_id=args.first_servo_id,
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
            logger.info("Current angles: %s", reader.get_angles())
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
