#!/usr/bin/env python3
"""01 - Hello, fake robot.

The smallest possible program: connect to a *simulated* Rizon, read state,
move the gripper, and go home. Runs with no hardware and no extra dependencies.

    python examples/01_fake_hello.py

To run the same code against a real arm, change ``backend="fake"`` to
``backend="flexiv_rdk"`` and set ``robot_sn`` (or load a YAML config), then make
sure the robot is in Remote/RDK mode in Flexiv Elements.
"""

import numpy as np

from flexiv_control import GripperCommand, Robot, RobotConfig


def main() -> None:
    robot = Robot(RobotConfig(backend="fake", control_hz=100.0))

    # ``with`` connects on entry and stops + disconnects on exit.
    with robot:
        state = robot.get_state()
        print("connected. joint positions q =", np.round(state.q, 3))
        print("TCP pose [x y z qw qx qy qz] =", np.round(state.tcp_pose, 3))

        print("opening gripper to 8 cm ...")
        robot.command_gripper(GripperCommand(width=0.08, force=20.0))
        print("gripper width is now", round(robot.get_state().gripper_width, 3), "m")

        print("homing ...")
        robot.home()
        print("done. final q =", np.round(robot.get_state().q, 3))


if __name__ == "__main__":
    main()
