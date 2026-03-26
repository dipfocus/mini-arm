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
    DEFAULT_BASE_JOINT_MAX_VEL = 2.0
    DEFAULT_WRIST_JOINT_MAX_VEL = 3.0
    DEFAULT_BASE_JOINT_MAX_ACC = 8.0
    DEFAULT_WRIST_JOINT_MAX_ACC = 12.0
    DEFAULT_GRIPPER_MAX_VEL = 3.0
    DEFAULT_GRIPPER_MAX_ACC = 12.0
    DEFAULT_MASTER_FILTER_TAU = 0.015
    DEFAULT_GRIPPER_FILTER_TAU = 0.02

    def __init__(
        self,
        channel="can0",
        control_hz=100.0,
        connect_speed_percent=5,
        base_joint_max_vel=DEFAULT_BASE_JOINT_MAX_VEL,
        wrist_joint_max_vel=DEFAULT_WRIST_JOINT_MAX_VEL,
        base_joint_max_acc=DEFAULT_BASE_JOINT_MAX_ACC,
        wrist_joint_max_acc=DEFAULT_WRIST_JOINT_MAX_ACC,
        gripper_max_vel=DEFAULT_GRIPPER_MAX_VEL,
        gripper_max_acc=DEFAULT_GRIPPER_MAX_ACC,
        master_filter_tau=DEFAULT_MASTER_FILTER_TAU,
        gripper_filter_tau=DEFAULT_GRIPPER_FILTER_TAU,
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
        if base_joint_max_acc <= 0.0:
            raise ValueError(
                f"base_joint_max_acc must be greater than 0, got {base_joint_max_acc}"
            )
        if wrist_joint_max_acc <= 0.0:
            raise ValueError(
                f"wrist_joint_max_acc must be greater than 0, got {wrist_joint_max_acc}"
            )
        if gripper_max_vel <= 0.0:
            raise ValueError(f"gripper_max_vel must be greater than 0, got {gripper_max_vel}")
        if gripper_max_acc <= 0.0:
            raise ValueError(f"gripper_max_acc must be greater than 0, got {gripper_max_acc}")
        if master_filter_tau < 0.0:
            raise ValueError(f"master_filter_tau must be non-negative, got {master_filter_tau}")
        if gripper_filter_tau < 0.0:
            raise ValueError(
                f"gripper_filter_tau must be non-negative, got {gripper_filter_tau}"
            )

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

        # === Velocity limiting parameters (can be adjusted per joint) ===
        # Use a lower limit on the larger base joints and a higher limit on the wrist joints.
        self.max_joint_vel = np.full(self.NERO_DOF, base_joint_max_vel, dtype=np.float64)
        self.max_joint_vel[3:] = wrist_joint_max_vel
        self.max_joint_acc = np.full(self.NERO_DOF, base_joint_max_acc, dtype=np.float64)
        self.max_joint_acc[3:] = wrist_joint_max_acc
        # Maximum gripper "normalized velocity" (/s), maximum change per second in 0~1 range
        self.max_gripper_vel = float(gripper_max_vel)
        self.max_gripper_acc = float(gripper_max_acc)
        self.master_filter_tau = float(master_filter_tau)
        self.gripper_filter_tau = float(gripper_filter_tau)

        self.lower_joint_limits, self.upper_joint_limits = self._ordered_joint_limits()

        # Command shaping state.
        self._filtered_desired_pos = self.home_joints.copy()
        self._last_cmd_pos = self.home_joints.copy()
        self._last_cmd_vel = np.zeros(self.NERO_DOF, dtype=np.float64)
        self._filtered_desired_grip = 0.0
        self._last_cmd_grip = 0.0
        self._last_cmd_grip_vel = 0.0
        logger.info(
            "Teleop zero aligned: master startup zero maps to follower joints %s",
            self.home_joints.tolist(),
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
        return self.home_joints + relative_joints

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

    @staticmethod
    def _low_pass(current, target, tau, dt):
        if tau <= 0.0:
            return target
        alpha = dt / (tau + dt)
        return current + alpha * (target - current)

    def send_cmd(self, master_angles_deg):
        # Smooth the discrete master-arm measurements before streaming them into JS mode.
        desired_pos = self._deg_to_rad_mapped(master_angles_deg)
        desired_grip = self._map_gripper(master_angles_deg)
        dt = self.control_dt

        self._filtered_desired_pos[:] = self._low_pass(
            self._filtered_desired_pos,
            desired_pos,
            self.master_filter_tau,
            dt,
        )
        desired_vel = np.clip(
            (self._filtered_desired_pos - self._last_cmd_pos) / dt,
            -self.max_joint_vel,
            self.max_joint_vel,
        )
        max_vel_step = self.max_joint_acc * dt
        cmd_vel = self._last_cmd_vel + np.clip(
            desired_vel - self._last_cmd_vel,
            -max_vel_step,
            max_vel_step,
        )
        cmd_pos = self._last_cmd_pos + cmd_vel * dt
        cmd_pos = np.clip(cmd_pos, self.lower_joint_limits, self.upper_joint_limits)
        cmd_vel = (cmd_pos - self._last_cmd_pos) / dt

        self._filtered_desired_grip = float(
            self._low_pass(
                self._filtered_desired_grip,
                desired_grip,
                self.gripper_filter_tau,
                dt,
            )
        )
        desired_grip_vel = float(
            np.clip(
                (self._filtered_desired_grip - self._last_cmd_grip) / dt,
                -self.max_gripper_vel,
                self.max_gripper_vel,
            )
        )
        max_grip_vel_step = self.max_gripper_acc * dt
        cmd_grip_vel = self._last_cmd_grip_vel + float(
            np.clip(
                desired_grip_vel - self._last_cmd_grip_vel,
                -max_grip_vel_step,
                max_grip_vel_step,
            )
        )
        grip_cmd = self._last_cmd_grip + cmd_grip_vel * dt
        grip_cmd = float(np.clip(grip_cmd, 0.0, 1.0))
        cmd_grip_vel = (grip_cmd - self._last_cmd_grip) / dt

        self.ctrl.move_js(cmd_pos.tolist(), blocking=False)
        self.ctrl.move_gripper(self._gripper_norm_to_width(grip_cmd), force=self.gripper_force)

        # Update state
        self._last_cmd_pos[:] = cmd_pos
        self._last_cmd_vel[:] = cmd_vel
        self._last_cmd_grip = grip_cmd
        self._last_cmd_grip_vel = cmd_grip_vel

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
        "--base-joint-max-acc",
        type=float,
        default=NeroTeleop.DEFAULT_BASE_JOINT_MAX_ACC,
        help="Acceleration limit for joints 1-3 in rad/s^2",
    )
    parser.add_argument(
        "--wrist-joint-max-acc",
        type=float,
        default=NeroTeleop.DEFAULT_WRIST_JOINT_MAX_ACC,
        help="Acceleration limit for joints 4-7 in rad/s^2",
    )
    parser.add_argument(
        "--gripper-max-vel",
        type=float,
        default=NeroTeleop.DEFAULT_GRIPPER_MAX_VEL,
        help='Velocity limit for gripper normalized command in 1/s',
    )
    parser.add_argument(
        "--gripper-max-acc",
        type=float,
        default=NeroTeleop.DEFAULT_GRIPPER_MAX_ACC,
        help="Acceleration limit for gripper normalized command in 1/s^2",
    )
    parser.add_argument(
        "--master-filter-tau",
        type=float,
        default=NeroTeleop.DEFAULT_MASTER_FILTER_TAU,
        help="Low-pass time constant for master joint targets in seconds",
    )
    parser.add_argument(
        "--gripper-filter-tau",
        type=float,
        default=NeroTeleop.DEFAULT_GRIPPER_FILTER_TAU,
        help="Low-pass time constant for master gripper target in seconds",
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

        # Create slave arm controller (with velocity limiting)
        teleop = NeroTeleop(
            channel=args.channel,
            control_hz=args.control_hz,
            connect_speed_percent=args.connect_speed_percent,
            base_joint_max_vel=args.base_joint_max_vel,
            wrist_joint_max_vel=args.wrist_joint_max_vel,
            base_joint_max_acc=args.base_joint_max_acc,
            wrist_joint_max_acc=args.wrist_joint_max_acc,
            gripper_max_vel=args.gripper_max_vel,
            gripper_max_acc=args.gripper_max_acc,
            master_filter_tau=args.master_filter_tau,
            gripper_filter_tau=args.gripper_filter_tau,
        )

        # Start reading thread
        t_reader = threading.Thread(
            target=servo_reader.read_loop,
            kwargs={"hz": args.control_hz},
            daemon=True,
        )
        t_reader.start()

        logger.info(
            "Teleoperation started on %s at %.1f Hz with joint vel %s rad/s, joint acc %s rad/s^2, filter tau %.3fs, press Ctrl+C to exit.",
            args.channel,
            args.control_hz,
            teleop.max_joint_vel.tolist(),
            teleop.max_joint_acc.tolist(),
            teleop.master_filter_tau,
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
