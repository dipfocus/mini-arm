#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from pathlib import Path
import threading
import time
import numpy as np
import sys

SRC_DIR = Path(__file__).resolve().parents[4]
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from uarm.scripts.Follower_Arm.Nero.arm_controller import ArmController
from uarm.scripts.Follower_Arm.Nero.servo_reader import ServoReader


class NeroTeleop:
    def __init__(self, interface="can0", control_hz=100.0, connect_speed_percent=5):
        if control_hz <= 0.0:
            raise ValueError(f"control_hz must be greater than 0, got {control_hz}")

        self.ctrl = ArmController(channel=interface)
        self.ctrl.connect(speed_percent=connect_speed_percent)
        self.ctrl.set_motion_mode("j")
        self.ctrl.move_to_home(blocking=True)

        current_joints = self.ctrl.get_joint_angles()
        if current_joints is None:
            raise RuntimeError("Failed to read joint angles after connecting to the Nero arm")

        self.dof = len(current_joints)
        self.control_dt = 1.0 / float(control_hz)

        # Master arm layout: 7 joint servos + 1 gripper servo.
        self.gripper_index = self.dof
        self.gripper_min_deg = -10.0
        self.gripper_max_deg = 30.0
        self.gripper_min_width = 0.0
        self.gripper_max_width = 0.1
        self.gripper_force = 1.0

        # Servo IDs 1-7 drive the 7 arm joints, and servo ID 8 drives the gripper.
        self.master_joint_count = min(self.gripper_index, self.dof)
        self.scale = np.ones(self.master_joint_count, dtype=np.float64)
        self.offset_rad = np.zeros(self.master_joint_count, dtype=np.float64)

        # === Velocity limiting parameters (can be adjusted per joint) ===
        # Maximum joint velocity (rad/s), default uniform upper limit (about 69 deg/s)
        self.max_joint_vel = np.array([1.2] * self.dof, dtype=np.float64)
        if self.dof > 3:
            self.max_joint_vel[3:] = 2.0
        # Maximum gripper "normalized velocity" (/s), maximum change per second in 0~1 range
        self.max_gripper_vel = 2.0

        # Velocity limiting state (target sent in previous cycle)
        self._last_cmd_pos = np.array(current_joints, dtype=np.float64)
        self._last_cmd_grip = 0.0

    def _deg_to_rad_mapped(self, master_angles_deg):
        """Master arm angle (degrees) -> Slave arm joint angle (radians)"""
        joints_rad = self._last_cmd_pos.copy()
        mapped_joint_count = min(len(master_angles_deg), self.master_joint_count)
        for j in range(mapped_joint_count):
            joints_rad[j] = np.deg2rad(master_angles_deg[j]) * self.scale[j] + self.offset_rad[j]

        if mapped_joint_count >= 6 and self.dof >= 6:
            joints_rad[4], joints_rad[5] = -joints_rad[5], joints_rad[4]

        return joints_rad

    def _map_gripper(self, master_angles_deg):
        if len(master_angles_deg) <= self.gripper_index:
            return self._last_cmd_grip
        grip_deg = master_angles_deg[self.gripper_index]
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
        delta = desired_pos - self._last_cmd_pos
        delta_clipped = np.clip(delta, -max_step, max_step)
        cmd_pos = self._last_cmd_pos + delta_clipped

        # === Gripper velocity limiting ===
        grip_delta = desired_grip - self._last_cmd_grip
        grip_step = self.max_gripper_vel * dt
        grip_cmd = self._last_cmd_grip + float(np.clip(grip_delta, -grip_step, grip_step))
        # Send command
        self.ctrl.move_j(cmd_pos.tolist(), blocking=False)
        self.ctrl.move_gripper(self._gripper_norm_to_width(grip_cmd), force=self.gripper_force)

        # Update state
        self._last_cmd_pos[:] = cmd_pos
        self._last_cmd_grip = grip_cmd

    def shutdown(self):
        try:
            self.ctrl.move_to_home(blocking=True)
        finally:
            self.ctrl.disconnect()


if __name__ == "__main__":
    teleop = None
    try:
        # Create master arm reader
        servo_reader = ServoReader(
            port="/dev/ttyUSB0",
            baudrate=115200,
            servo_count=8,
            first_servo_id=1,
        )

        # Create slave arm controller (with velocity limiting)
        teleop = NeroTeleop(interface="can0", control_hz=100.0)

        # Start reading thread
        t_reader = threading.Thread(target=servo_reader.read_loop, kwargs={"hz": 100}, daemon=True)
        t_reader.start()

        print("[Main] Teleoperation started, press Ctrl+C to exit.")
        dt = teleop.control_dt
        while True:
            master_angles = servo_reader.get_angles()
            teleop.send_cmd(master_angles)
            time.sleep(dt)

    except KeyboardInterrupt:
        print("\n[Main] Interrupted, robot arm returning to home...")
    finally:
        if teleop is not None:
            teleop.shutdown()
        print("[Main] Exited.")
