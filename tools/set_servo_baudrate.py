#!/usr/bin/env python3
"""Set the Zhongling servo bus baudrate over the serial text protocol."""

from __future__ import annotations

import argparse
import logging
import time

import serial


logger = logging.getLogger(__name__)

SUPPORTED_BAUDRATES = {
    9600: 0,
    19200: 1,
    38400: 2,
    57600: 3,
    115200: 4,
    128000: 5,
    256000: 6,
    1000000: 7,
}
RESPONSE_TERMINATOR = b"!"
BAUDRATE_QUERY = "#000PVER!"


def open_serial(port: str, baudrate: int, timeout: float) -> serial.Serial:
    return serial.Serial(
        port=port,
        baudrate=baudrate,
        timeout=timeout,
        write_timeout=timeout,
    )


def read_response(ser: serial.Serial) -> str:
    response = ser.read_until(RESPONSE_TERMINATOR)
    if ser.in_waiting:
        response += ser.read(ser.in_waiting)
    return response.decode("ascii", errors="ignore").strip()


def send_command(ser: serial.Serial, command: str) -> str:
    ser.reset_input_buffer()
    ser.write(command.encode("ascii"))
    ser.flush()
    return read_response(ser)


def is_protocol_response(response: str) -> bool:
    return response.startswith("#000") and response.endswith("!")


def verify_baudrate(
    *,
    port: str,
    baudrate: int,
    timeout: float,
    attempts: int,
    settle_time: float,
) -> tuple[bool, str]:
    last_response = ""
    with open_serial(port, baudrate, timeout) as ser:
        for attempt in range(1, attempts + 1):
            response = send_command(ser, BAUDRATE_QUERY)
            if is_protocol_response(response):
                logger.info(
                    "Verification succeeded on attempt %d/%d: %s",
                    attempt,
                    attempts,
                    response,
                )
                return True, response
            if response:
                logger.warning(
                    "Unexpected response on attempt %d/%d at %d baud: %s",
                    attempt,
                    attempts,
                    baudrate,
                    response,
                )
            if attempt < attempts:
                time.sleep(settle_time)
            last_response = response
    return False, last_response


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Set the Zhongling servo bus baudrate with the #000PBDx! command. "
            "Example: python3 tools/set_servo_baudrate.py 9600"
        )
    )
    parser.add_argument(
        "target_baudrate",
        type=int,
        choices=sorted(SUPPORTED_BAUDRATES),
        help="Target baudrate to apply.",
    )
    parser.add_argument(
        "--port",
        default="/dev/ttyUSB0",
        help="Serial port path.",
    )
    parser.add_argument(
        "--current-baudrate",
        type=int,
        default=115200,
        help="Current baudrate used to send the change command.",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=0.1,
        help="Serial read/write timeout in seconds.",
    )
    parser.add_argument(
        "--settle-time",
        type=float,
        default=0.2,
        help="Wait time after sending the command before verifying the new baudrate.",
    )
    parser.add_argument(
        "--verify-attempts",
        type=int,
        default=5,
        help="Number of verification attempts after switching baudrate.",
    )
    parser.add_argument(
        "--no-verify",
        action="store_true",
        help="Skip reopening the port at the new baudrate for verification.",
    )
    args = parser.parse_args()

    if args.timeout <= 0.0:
        parser.error("--timeout must be greater than 0")
    if args.settle_time < 0.0:
        parser.error("--settle-time must be greater than or equal to 0")
    if args.verify_attempts <= 0:
        parser.error("--verify-attempts must be greater than 0")

    logging.basicConfig(
        level=logging.INFO,
        format="[%(asctime)s] [%(levelname)s] [%(filename)s:%(lineno)d] %(message)s",
        datefmt="%H:%M:%S",
    )

    command_code = SUPPORTED_BAUDRATES[args.target_baudrate]
    command = f"#000PBD{command_code}!"
    expected_ack = f"#000PBD{args.target_baudrate}!"
    received_expected_ack = False

    logger.info(
        "Setting servo baudrate on %s: %d -> %d using %s",
        args.port,
        args.current_baudrate,
        args.target_baudrate,
        command,
    )

    try:
        with open_serial(args.port, args.current_baudrate, args.timeout) as ser:
            response = send_command(ser, command)
            if response:
                logger.info("Immediate response at old baudrate: %s", response)
                received_expected_ack = response == expected_ack
                if not received_expected_ack:
                    logger.warning(
                        "Immediate response did not match the expected acknowledgement %s",
                        expected_ack,
                    )
            else:
                logger.warning(
                    "No immediate response at %d. The device may have switched baudrate "
                    "before the reply was read.",
                    args.current_baudrate,
                )
    except serial.SerialException as exc:
        logger.error("Failed to access serial port %s: %s", args.port, exc)
        return 1

    if not args.no_verify:
        time.sleep(args.settle_time)
        try:
            verified, verify_response = verify_baudrate(
                port=args.port,
                baudrate=args.target_baudrate,
                timeout=args.timeout,
                attempts=args.verify_attempts,
                settle_time=args.settle_time,
            )
        except serial.SerialException as exc:
            logger.error(
                "Baudrate command sent, but verification failed to open %s at %d: %s",
                args.port,
                args.target_baudrate,
                exc,
            )
            return 1

        if not verified:
            logger.error(
                "No response received when reopening %s at %d. "
                "Please verify the cabling and the current baudrate manually.",
                args.port,
                args.target_baudrate,
            )
            return 1

        logger.info("Servo bus is responding at the new baudrate %d", args.target_baudrate)
        if received_expected_ack:
            logger.info("Device acknowledged the baudrate change: %s", expected_ack)
        return 0

    logger.info(
        "Baudrate change command sent. Reconnect using %d to continue communicating.",
        args.target_baudrate,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
