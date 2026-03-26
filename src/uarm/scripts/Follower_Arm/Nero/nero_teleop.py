#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
from pathlib import Path
import logging
import threading
import time
import numpy as np
import sys

SRC_DIR = Path(__file__).resolve().parents[4]
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from uarm.scripts.Follower_Arm.Nero.arm_controller import ArmController
from uarm.scripts.Follower_Arm.Nero.log_utils import configure_logging
from uarm.scripts.Follower_Arm.Nero.servo_reader import ServoReader


logger = logging.getLogger(__name__)


class NeroTeleop:
    NERO_DOF = 7
    MASTER_SERVO_COUNT = 8
    GRIPPER_INDEX = NERO_DOF
    # Default calibration for the current master arm build:
    # joint 2 servo direction is opposite to the follower arm.
    DEFAULT_MASTER_JOINT_SCALES = (
        1.0,
        -1.0,
        1.0,
        1.0,
        1.0,
        1.0,
        -1.0,
    )

    def __init__(
        self,
        channel="can0",
        control_hz=100.0,
        connect_speed_percent=5,
    ):
        if control_hz <= 0.0:
            raise ValueError(f"control_hz must be greater than 0, got {control_hz}")

        self.ctrl = ArmController(channel=channel)
        self.ctrl.connect(speed_percent=connect_speed_percent)
        self.ctrl.move_to_home(blocking=True)

        current_joints = self.ctrl.get_joint_angles()
        if current_joints is None:
            raise RuntimeError("Failed to read joint angles after connecting to the Nero arm")

        if len(current_joints) != self.NERO_DOF:
            raise RuntimeError(
                f"NeroTeleop expects {self.NERO_DOF} follower-arm joints, got {len(current_joints)}"
            )
        self.control_dt = 1.0 / float(control_hz)
        # Treat the startup home pose as the teleoperation zero target.
        self.home_joints = np.array(current_joints, dtype=np.float64)

        # Master arm layout: 7 joint servos + 1 gripper servo.
        self.gripper_min_deg = -10.0
        self.gripper_max_deg = 30.0
        self.gripper_min_width = 0.0
        self.gripper_max_width = 0.1
        self.gripper_force = 1.0

        self.master_joint_scales = self._normalize_master_joint_scales(
            self.DEFAULT_MASTER_JOINT_SCALES
        )
        self.lower_joint_limits, self.upper_joint_limits = self._ordered_joint_limits()
        logger.info(
            "Teleop zero aligned: master startup zero maps to follower joints %s with scales %s",
            self.home_joints.tolist(),
            self.master_joint_scales.tolist(),
        )

    def _validate_master_angles(self, master_angles_deg):
        actual_count = len(master_angles_deg)
        if actual_count != self.MASTER_SERVO_COUNT:
            raise ValueError(
                "Expected 8 master servo angles (7 joints + 1 gripper), "
                f"got {actual_count}"
            )

    def _deg_to_rad_mapped(self, master_angles_deg):
        """Master relative angle (degrees) -> follower home-referenced joint angle (radians)."""
        self._validate_master_angles(master_angles_deg)
        relative_joints = np.deg2rad(master_angles_deg[:self.NERO_DOF]).astype(
            np.float64,
            copy=False,
        )
        relative_joints *= self.master_joint_scales
        return self.home_joints + relative_joints

    def _normalize_master_joint_scales(self, master_joint_scales):
        scales = np.asarray(master_joint_scales, dtype=np.float64)
        if scales.shape != (self.NERO_DOF,):
            raise ValueError(
                f"Expected {self.NERO_DOF} master joint scales, got shape {scales.shape}"
            )
        if np.any(np.isclose(scales, 0.0)):
            raise ValueError("master joint scales must be non-zero")
        return scales

    def _map_gripper(self, master_angles_deg):
        self._validate_master_angles(master_angles_deg)
        grip_deg = master_angles_deg[self.GRIPPER_INDEX]
        grip_norm = (grip_deg - self.gripper_min_deg) / max(1e-6, self.gripper_max_deg - self.gripper_min_deg)
        return float(np.clip(grip_norm, 0.0, 1.0))

    def _gripper_norm_to_width(self, grip_norm):
        return float(
            np.clip(
                self.gripper_min_width
                + grip_norm * (self.gripper_max_width - self.gripper_min_width),
                self.gripper_min_width,
                self.gripper_max_width,
            )
        )

    def _ordered_joint_limits(self):
        lower = np.full(self.NERO_DOF, -np.inf, dtype=np.float64)
        upper = np.full(self.NERO_DOF, np.inf, dtype=np.float64)
        limits = self.ctrl.joint_limits
        for index, joint_name in enumerate(self.ctrl.joint_names[:self.NERO_DOF]):
            bounds = limits.get(joint_name)
            if bounds is None:
                continue
            lower[index], upper[index] = bounds
        return lower, upper

    def send_cmd(self, master_angles_deg):
        desired_pos = self._deg_to_rad_mapped(master_angles_deg)
        desired_pos = np.clip(desired_pos, self.lower_joint_limits, self.upper_joint_limits)
        desired_grip = self._map_gripper(master_angles_deg)
        self.ctrl.move_j(desired_pos.tolist(), blocking=False)
        self.ctrl.move_gripper(
            self._gripper_norm_to_width(desired_grip),
            force=self.gripper_force,
        )

    def shutdown(self):
        try:
            self.ctrl.move_to_home(blocking=True)
        finally:
            self.ctrl.disconnect()


def _parse_args():
    parser = argparse.ArgumentParser(description="Teleoperate the Nero follower arm.")
    parser.add_argument("--channel", default="can0", help="CAN channel for the Nero arm")
    parser.add_argument(
        "--control-hz",
        type=float,
        default=100.0,
        help="Teleoperation loop frequency in Hz",
    )
    parser.add_argument(
        "--connect-speed-percent",
        type=int,
        default=5,
        help="Initial vendor speed percent used during connect and homing",
    )
    parser.add_argument("--port", default="/dev/ttyUSB0", help="Serial port for the master arm")
    parser.add_argument(
        "--baudrate",
        type=int,
        default=115200,
        help="Baudrate for the master arm serial connection",
    )
    parser.add_argument(
        "--response-timeout",
        type=float,
        default=0.01,
        help="Per-read serial timeout for the master arm in seconds.",
    )
    return parser.parse_args()


if __name__ == "__main__":
    configure_logging()
    teleop = None
    args = _parse_args()
    try:
        # Create master arm reader
        servo_reader = ServoReader(
            port=args.port,
            baudrate=args.baudrate,
            servo_count=NeroTeleop.MASTER_SERVO_COUNT,
            first_servo_id=1,
            response_timeout=args.response_timeout,
        )

        # Create slave arm controller in direct follow mode.
        teleop = NeroTeleop(
            channel=args.channel,
            control_hz=args.control_hz,
            connect_speed_percent=args.connect_speed_percent,
        )

        # Start reading thread
        t_reader = threading.Thread(
            target=servo_reader.read_loop,
            kwargs={"hz": args.control_hz},
            daemon=True,
        )
        t_reader.start()

        logger.info(
            "Teleoperation started on %s at %.1f Hz in direct follow mode, press Ctrl+C to exit.",
            args.channel,
            args.control_hz,
        )
        next_tick = time.monotonic()
        while True:
            master_angles = servo_reader.get_angles()
            teleop.send_cmd(master_angles)
            next_tick += teleop.control_dt
            remaining = next_tick - time.monotonic()
            if remaining > 0.0:
                time.sleep(remaining)
            else:
                next_tick = time.monotonic()

    except KeyboardInterrupt:
        logger.info("Interrupted, robot arm returning to home...")
    finally:
        if teleop is not None:
            teleop.shutdown()
        logger.info("Exited.")
