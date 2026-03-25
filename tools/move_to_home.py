#!/usr/bin/env python3
# coding=utf-8
"""
Tool to move the robot to the home position.

Usage:
    python3 tools/move_to_home.py
"""

import argparse
import logging
import sys
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_DIR = REPO_ROOT / "src"

if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

logger = logging.getLogger(__name__)


def main() -> int:
    parser = argparse.ArgumentParser(description="Nero Workcell - Move to Home")
    parser.add_argument("--channel", type=str, default="can0", help="CAN channel")
    parser.add_argument("--speed", type=int, default=5, help="Movement speed percent")
    args = parser.parse_args()

    from uarm.scripts.Follower_Arm.Nero.arm_controller import ArmController

    logging.basicConfig(
        level=logging.INFO,
        format='[%(asctime)s] [%(levelname)s] [%(filename)s:%(lineno)d] %(message)s',
        datefmt='%H:%M:%S'
    )

    controller = ArmController(channel=args.channel)
    if not controller.connect(speed_percent=args.speed):
        logger.error("Failed to connect to robot")
        return 1

    try:
        controller.move_to_home(blocking=True)
        logger.info("Robot moved to home position")
    except Exception as e:
        logger.error(f"Failed to move home: {e}")
        return 1
    finally:
        controller.disconnect()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
