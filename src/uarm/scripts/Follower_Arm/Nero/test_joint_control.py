#!/usr/bin/env python3

import sys
import time
from pathlib import Path
from typing import TYPE_CHECKING

import click
import numpy as np

SRC_DIR = Path(__file__).resolve().parents[4]
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

if TYPE_CHECKING:
    from uarm.scripts.Follower_Arm.Nero.arm_controller import ArmController

from uarm.scripts.Follower_Arm.Nero.log_utils import configure_logging


def ease_in_out_quad(t: float) -> float:
    t *= 2.0
    if t < 1.0:
        return t * t / 2.0
    t -= 1.0
    return -(t * (t - 2.0) - 1.0) / 2.0


def ordered_joint_limits(controller: "ArmController", joint_count: int) -> tuple[np.ndarray, np.ndarray]:
    lower = np.full(joint_count, -np.inf, dtype=np.float64)
    upper = np.full(joint_count, np.inf, dtype=np.float64)
    limits = controller.joint_limits
    for index, joint_name in enumerate(controller.joint_names[:joint_count]):
        bounds = limits.get(joint_name)
        if bounds is None:
            continue
        lower[index], upper[index] = bounds
    return lower, upper


def run_segment(
    controller: "ArmController",
    home_joints: np.ndarray,
    joint_delta: np.ndarray,
    lower_limits: np.ndarray,
    upper_limits: np.ndarray,
    *,
    steps: int,
    control_dt: float,
    gripper_width: float,
    reverse: bool = False,
) -> None:
    for index in range(steps):
        progress = index / float(steps - 1)
        if reverse:
            progress = 1.0 - progress
        blend = ease_in_out_quad(progress)

        command = home_joints + blend * joint_delta
        command = np.clip(command, lower_limits, upper_limits)
        controller.move_j(command.tolist(), blocking=False)
        controller.move_gripper(width=gripper_width * blend, force=1.0)
        time.sleep(control_dt)


@click.command()
@click.option("--channel", default="can0", show_default=True, help="CAN channel name.")
@click.option("--steps", default=1500, show_default=True, type=int, help="Interpolation steps per segment.")
@click.option("--control-hz", default=100.0, show_default=True, type=float, help="Control frequency in Hz.")
@click.option("--speed-percent", default=5, show_default=True, type=click.IntRange(0, 100), help="Robot motion speed percent.")
@click.option(
    "--gripper-width",
    default=0.08,
    show_default=True,
    type=click.FloatRange(0.0, 0.1),
    help="Maximum gripper width in meters.",
)
def main(
    channel: str,
    steps: int,
    control_hz: float,
    speed_percent: int,
    gripper_width: float,
) -> None:
    from uarm.scripts.Follower_Arm.Nero.arm_controller import ArmController

    configure_logging()

    if steps < 2:
        raise click.BadParameter("steps must be at least 2", param_hint="steps")
    if control_hz <= 0.0:
        raise click.BadParameter("control_hz must be greater than 0", param_hint="control_hz")

    control_dt = 1.0 / float(control_hz)

    controller = ArmController(channel=channel)

    try:
        controller.connect(speed_percent=speed_percent)
        controller.set_motion_mode("j")
        controller.move_to_home(blocking=True)

        current_joints = controller.get_joint_angles()
        if current_joints is None:
            raise click.ClickException("Failed to read joint angles from the Nero arm")

        home_joints = np.array(current_joints, dtype=np.float64)
        lower_limits, upper_limits = ordered_joint_limits(controller, home_joints.size)

        requested_delta = np.zeros_like(home_joints)
        active_joint_count = min(4, home_joints.size)
        requested_delta[:active_joint_count] = np.array([1.0, 2.0, 2.0, 1.5], dtype=np.float64)[
            :active_joint_count
        ]

        target_joints = np.clip(home_joints + requested_delta, lower_limits, upper_limits)
        joint_delta = target_joints - home_joints

        run_segment(
            controller,
            home_joints,
            joint_delta,
            lower_limits,
            upper_limits,
            steps=steps,
            control_dt=control_dt,
            gripper_width=gripper_width,
        )
        run_segment(
            controller,
            home_joints,
            joint_delta,
            lower_limits,
            upper_limits,
            steps=steps,
            control_dt=control_dt,
            gripper_width=gripper_width,
            reverse=True,
        )
        controller.move_to_home(blocking=True)
    finally:
        if controller.is_connected():
            controller.disconnect()


if __name__ == "__main__":
    main()
