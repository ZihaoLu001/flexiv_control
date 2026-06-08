#!/usr/bin/env python3
"""04 - Model-predictive control / high-rate closed loop.

An MPC controller solves for an optimal action given the latest state and
streams a fresh setpoint every tick. This stack supports that directly: read
state, compute a target pose (or delta), send it, repeat -- at whatever rate
your solver runs.

    python examples/04_mpc_loop.py

Two equivalent ways to command a single setpoint per tick:
  * ``robot.servo_cartesian_pose(pose, duration=dt)`` -- absolute TCP pose
  * ``robot.servo_cartesian_delta(delta, duration=dt)`` -- relative twist*dt

Both go through the safety filter. For the lowest-latency, always-on streaming
loop (the Python analogue of the C++ 1 kHz daemon) see ``ReactiveServoLoop`` and
example 05; for true 1 kHz hard-real-time, run the optional C++ ``rt_server``.
"""

import time

import numpy as np

from flexiv_control import Robot, RobotConfig


def trivial_mpc(state_pose: np.ndarray, goal_xyz: np.ndarray, dt: float) -> np.ndarray:
    """A stand-in 'solver': proportional step toward a Cartesian goal.

    Returns an absolute target pose. A real MPC would optimise a horizon of
    actions subject to dynamics + constraints and return the first one.
    """
    err = goal_xyz - state_pose[:3]
    k = 2.0  # proportional gain (1/s)
    step = np.clip(k * err * dt, -0.01, 0.01)
    target = state_pose.copy()
    target[:3] = state_pose[:3] + step
    return target


def main() -> None:
    hz = 100.0
    dt = 1.0 / hz
    robot = Robot(RobotConfig(backend="fake", control_hz=hz))

    with robot:
        robot.acquire_lease("mpc")
        robot.start_cartesian_impedance(realtime=False)

        goal = np.array([0.55, 0.10, 0.30])
        print(f"driving TCP to goal {goal} at {hz:.0f} Hz ...")

        t_loop = time.perf_counter()
        for k in range(200):
            state = robot.get_state()
            target_pose = trivial_mpc(state.tcp_pose, goal, dt)
            robot.servo_cartesian_pose(target_pose, duration=dt)

            if k % 40 == 0:
                err = np.linalg.norm(goal - state.tcp_position)
                print(f"  k={k:3d}  pos={np.round(state.tcp_position, 3)}  err={err*1000:5.1f} mm")

            # keep loop rate
            t_loop += dt
            sleep = t_loop - time.perf_counter()
            if sleep > 0:
                time.sleep(sleep)

        final = robot.get_state().tcp_position
        print(f"final pos {np.round(final, 3)}  (goal {goal}, err "
              f"{np.linalg.norm(goal - final)*1000:.1f} mm)")


if __name__ == "__main__":
    main()
