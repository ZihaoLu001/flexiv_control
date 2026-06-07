#!/usr/bin/env python3
"""03 - Reinforcement learning with the Gymnasium env.

``FlexivRealEnv`` exposes the arm as a standard Gymnasium environment so any RL
or imitation-learning code (SERL/HIL-SERL, LeRobot, stable-baselines3, your own
actor-learner) can drive the robot through the same safe, leased control path
the rest of the stack uses.

    action:      [dx, dy, dz, droll, dpitch, dyaw, gripper] in [-1, 1]
    observation: [q(7), dq(7), tcp_pose(7), wrench(6), gripper_width(1)]  -> 28

This example runs a random policy on the fake backend, then shows the SERL/
HIL-SERL pattern where a human SpaceMouse operator can *intervene* and overwrite
the policy action (those corrections are exactly what HIL-SERL learns from).

    python examples/03_rl_gym_env.py

With real hardware: ``make_env(config=RobotConfig.load("rizon4s_lab"))`` and the
``rl_conservative`` safety profile is applied automatically.
"""

import numpy as np

from flexiv_control import RobotConfig
from flexiv_control.envs import make_env


def main() -> None:
    env = make_env(
        robot=None,
        config=RobotConfig(backend="fake", control_hz=20.0),
        control_hz=20.0,
        max_episode_steps=20,
        safety_profile="rl_conservative",
    )

    obs, info = env.reset()
    print("obs dim:", obs.shape[0], "| action dim:", env.action_space.shape[0])

    # --- plain RL rollout ---------------------------------------------------
    ep_return = 0.0
    for t in range(10):
        action = env.action_space.sample()  # replace with policy(obs)
        obs, reward, terminated, truncated, info = env.step(action)
        ep_return += reward
        if terminated or truncated:
            break
    print(f"random rollout: {t+1} steps, return={ep_return:.3f}")

    # --- HIL-SERL style human intervention ----------------------------------
    # The SpaceMouse can overwrite the policy's action. When the operator
    # touches the puck, ``intervened`` is True and that action is what you store
    # as a human correction for the learner.
    try:
        from flexiv_control.teleop import ScriptedSpaceMouseSource, SpaceMouseTeleop

        teleop = SpaceMouseTeleop(robot=env.robot, source=ScriptedSpaceMouseSource())
        obs, _ = env.reset()
        for t in range(10):
            policy_action = np.zeros(env.action_space.shape[0], dtype=np.float32)
            action, intervened = teleop.intervention(policy_action)
            obs, reward, terminated, truncated, info = env.step(action)
            if intervened:
                print(f"  step {t}: human override -> store as correction")
            if terminated or truncated:
                break
    except Exception as exc:  # teleop extra not installed, or no device
        print("(skipped intervention demo:", exc, ")")

    env.close()


if __name__ == "__main__":
    main()
