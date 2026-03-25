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
    DEFAULT_BASE_JOINT_MAX_VEL = 1.2
    DEFAULT_WRIST_JOINT_MAX_VEL = 2.0
    DEFAULT_GRIPPER_MAX_VEL = 2.0

    def __init__(
        self,
        channel="can0",
        control_hz=100.0,
        connect_speed_percent=5,
        base_joint_max_vel=DEFAULT_BASE_JOINT_MAX_VEL,
        wrist_joint_max_vel=DEFAULT_WRIST_JOINT_MAX_VEL,
        gripper_max_vel=DEFAULT_GRIPPER_MAX_VEL,
    ):
        if control_hz <= 0.0:
            raise ValueError(f"control_hz must be greater than 0, got {control_hz}")
        if base_joint_max_vel <= 0.0:
            raise ValueError(
                f"base_joint_max_vel must be greater than 0, got {base_joint_max_vel}"
            )
        if wrist_joint_max_vel <= 0.0:
            raise ValueError(
                f"wrist_joint_max_vel must be greater than 0, got {wrist_joint_max_vel}"
            )
        if gripper_max_vel <= 0.0:
            raise ValueError(f"gripper_max_vel must be greater than 0, got {gripper_max_vel}")

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

        # Master arm layout: 7 joint servos + 1 gripper servo.
        self.gripper_min_deg = -10.0
        self.gripper_max_deg = 30.0
        self.gripper_min_width = 0.0
        self.gripper_max_width = 0.1
        self.gripper_force = 1.0

        # === Velocity limiting parameters (can be adjusted per joint) ===
        # Use a lower limit on the larger base joints and a higher limit on the wrist joints.
        self.max_joint_vel = np.full(self.NERO_DOF, base_joint_max_vel, dtype=np.float64)
        self.max_joint_vel[3:] = wrist_joint_max_vel
        # Maximum gripper "normalized velocity" (/s), maximum change per second in 0~1 range
        self.max_gripper_vel = float(gripper_max_vel)

        # Velocity limiting state (target sent in previous cycle)
        self._last_cmd_pos = np.array(current_joints, dtype=np.float64)
        self._last_cmd_grip = 0.0

    def _validate_master_angles(self, master_angles_deg):
        actual_count = len(master_angles_deg)
        if actual_count != self.MASTER_SERVO_COUNT:
            raise ValueError(
                "Expected 8 master servo angles (7 joints + 1 gripper), "
                f"got {actual_count}"
            )

    def _deg_to_rad_mapped(self, master_angles_deg):
        """Master arm angle (degrees) -> Slave arm joint angle (radians)"""
        self._validate_master_angles(master_angles_deg)
        return np.deg2rad(master_angles_deg[:self.NERO_DOF]).astype(np.float64, copy=False)

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

    def send_cmd(self, master_angles_deg):
        # Desired pose
        desired_pos = self._deg_to_rad_mapped(master_angles_deg)
        desired_grip = self._map_gripper(master_angles_deg)
        dt = self.control_dt

        # === Joint velocity limiting: limit maximum step per cycle ===
        max_step = self.max_joint_vel * dt
        cmd_pos = self._last_cmd_pos + np.clip(
            desired_pos - self._last_cmd_pos,
            -max_step,
            max_step,
        )

        # === Gripper velocity limiting ===
        grip_delta = desired_grip - self._last_cmd_grip
        grip_step = self.max_gripper_vel * dt
        grip_cmd = self._last_cmd_grip + float(np.clip(grip_delta, -grip_step, grip_step))
        # Send command
        self.ctrl.move_js(cmd_pos.tolist(), blocking=False)
        self.ctrl.move_gripper(self._gripper_norm_to_width(grip_cmd), force=self.gripper_force)

        # Update state
        self._last_cmd_pos[:] = cmd_pos
        self._last_cmd_grip = grip_cmd

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
    parser.add_argument(
        "--base-joint-max-vel",
        type=float,
        default=NeroTeleop.DEFAULT_BASE_JOINT_MAX_VEL,
        help="Velocity limit for joints 1-3 in rad/s",
    )
    parser.add_argument(
        "--wrist-joint-max-vel",
        type=float,
        default=NeroTeleop.DEFAULT_WRIST_JOINT_MAX_VEL,
        help="Velocity limit for joints 4-7 in rad/s",
    )
    parser.add_argument(
        "--gripper-max-vel",
        type=float,
        default=NeroTeleop.DEFAULT_GRIPPER_MAX_VEL,
        help='Velocity limit for gripper normalized command in 1/s',
    )
    parser.add_argument("--port", default="/dev/ttyUSB0", help="Serial port for the master arm")
    parser.add_argument(
        "--baudrate",
        type=int,
        default=115200,
        help="Baudrate for the master arm serial connection",
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
        )

        # Create slave arm controller (with velocity limiting)
        teleop = NeroTeleop(
            channel=args.channel,
            control_hz=args.control_hz,
            connect_speed_percent=args.connect_speed_percent,
            base_joint_max_vel=args.base_joint_max_vel,
            wrist_joint_max_vel=args.wrist_joint_max_vel,
            gripper_max_vel=args.gripper_max_vel,
        )

        # Start reading thread
        t_reader = threading.Thread(
            target=servo_reader.read_loop,
            kwargs={"hz": args.control_hz},
            daemon=True,
        )
        t_reader.start()

        logger.info(
            "Teleoperation started on %s at %.1f Hz with joint limits %s rad/s, press Ctrl+C to exit.",
            args.channel,
            args.control_hz,
            teleop.max_joint_vel.tolist(),
        )
        dt = teleop.control_dt
        while True:
            master_angles = servo_reader.get_angles()
            teleop.send_cmd(master_angles)
            time.sleep(dt)

    except KeyboardInterrupt:
        logger.info("Interrupted, robot arm returning to home...")
    finally:
        if teleop is not None:
            teleop.shutdown()
        logger.info("Exited.")
