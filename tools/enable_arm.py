#!/usr/bin/env python3
"""Enable the Nero robot arm and leave it in normal mode."""

from __future__ import annotations

import argparse
import logging
import time

from pyAgxArm import AgxArmFactory, create_agx_arm_config


logger = logging.getLogger(__name__)


def wait_until_enabled(robot, *, timeout: float, poll_interval: float) -> None:
    deadline = time.monotonic() + timeout
    retry_count = 0

    while not robot.enable():
        retry_count += 1
        if time.monotonic() >= deadline:
            raise TimeoutError(f"Failed to enable robot within {timeout:.2f}s")

        if retry_count == 1 or retry_count % 100 == 0:
            logger.info("Robot not enabled yet, retrying...")
        time.sleep(poll_interval)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Enable the Nero arm on the specified CAN channel."
    )
    parser.add_argument("--channel", default="can0", help="CAN channel")
    parser.add_argument(
        "--timeout",
        type=float,
        default=5.0,
        help="Maximum time to wait for enable success, in seconds",
    )
    parser.add_argument(
        "--poll-interval",
        type=float,
        default=0.01,
        help="Polling interval for enable checks, in seconds",
    )
    args = parser.parse_args()

    if args.timeout <= 0.0:
        parser.error("--timeout must be greater than 0")
    if args.poll_interval <= 0.0:
        parser.error("--poll-interval must be greater than 0")

    logging.basicConfig(
        level=logging.INFO,
        format="[%(asctime)s] [%(levelname)s] [%(filename)s:%(lineno)d] %(message)s",
        datefmt="%H:%M:%S",
    )

    cfg = create_agx_arm_config(robot="nero", comm="can", channel=args.channel)
    if cfg is None:
        logger.error("Failed to create AGX config for channel %s", args.channel)
        return 1

    robot = AgxArmFactory.create_arm(cfg)
    if robot is None:
        logger.error("Failed to create AGX arm instance")
        return 1

    try:
        robot.connect()
        robot.set_normal_mode()
        wait_until_enabled(
            robot,
            timeout=args.timeout,
            poll_interval=args.poll_interval,
        )
        logger.info("Robot enabled on channel %s", args.channel)
        return 0
    except Exception as exc:
        logger.error("Failed to enable robot: %s", exc)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
