#!/usr/bin/env python3
"""06 - Record data in the LeRobot format.

The LeRobot adapter makes a Flexiv arm look like a ``lerobot`` robot, so you get
HuggingFace's data-collection, training, visualization and dataset tooling
(``LeRobotDataset``, MP4 + Parquet) "for free". This is the single biggest lever
for sharing data and policies with the wider community.

    pip install "flexiv-control[lerobot]"
    python examples/06_lerobot_record.py

This example shows the adapter's ``connect / get_observation / send_action``
surface and a minimal record loop. If ``lerobot`` is not installed it falls back
to printing the frames it *would* log, so it still runs on the fake backend.

    observation_features: q, dq, tcp_pose, wrench, gripper_width
    action_features:      action(7) = [dx, dy, dz, droll, dpitch, dyaw, gripper]

NOTE: feature names/dtypes are marked ``# VERIFY:`` in the adapter -- pin them to
your installed LeRobot version before recording a real dataset.
"""

import numpy as np

from flexiv_control import RobotConfig
from flexiv_control.adapters import LeRobotFlexivAdapter


def main() -> None:
    adapter = LeRobotFlexivAdapter(
        config=RobotConfig(backend="fake", control_hz=20.0),
        owner="lerobot_record",
    )
    print("observation_features:", list(adapter.observation_features.keys()))
    print("action_features     :", list(adapter.action_features.keys()))

    adapter.connect()
    try:
        frames = []
        for t in range(10):
            obs = adapter.get_observation()
            # A real script would call your policy here. The action is a single
            # 7-vector [dx, dy, dz, droll, dpitch, dyaw, gripper] in [-1, 1];
            # here we send a small +x nudge with the gripper open.
            action = np.array([0.3, 0, 0, 0, 0, 0, 1.0], dtype=np.float32)
            adapter.send_action(action)
            frames.append((obs, action))

        print(f"collected {len(frames)} frames")
        try:
            import lerobot  # noqa: F401

            print("lerobot is installed: wrap `frames` in a LeRobotDataset to save "
                  "(see docs/integration_rl.md).")
        except Exception:
            print("(lerobot not installed -> frames printed only; install "
                  "flexiv-control[lerobot] to write a real dataset.)")
            print("  example frame 0 obs keys:", list(frames[0][0].keys()))
    finally:
        adapter.disconnect()


if __name__ == "__main__":
    main()
