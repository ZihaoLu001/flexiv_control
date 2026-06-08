#!/usr/bin/env python3
"""06 - Drive the LeRobot adapter surface (demo).

The LeRobot adapter makes a Flexiv arm look like a ``lerobot`` robot, so it plugs
into HuggingFace's data-collection, training, and visualization tooling. This
example demonstrates the adapter's record-loop surface; it does **not** itself
write a ``LeRobotDataset`` -- turning the collected frames into a dataset
(MP4 + Parquet) is a thin step against your installed ``lerobot`` version (whose
dataset API moves between releases), sketched in the notes below.

    pip install "flexiv-control[lerobot]"
    python examples/06_lerobot_record.py

This example shows the adapter's ``connect / get_observation / send_action``
surface and a minimal record loop. If ``lerobot`` is not installed it falls back
to printing the frames it *would* log, so it still runs on the fake backend.

    observation_features: q.0..6, dq.0..6, tcp_pose.0..6, wrench.0..5, gripper_width
    action_features:      dx, dy, dz, drx, dry, drz, gripper  (rotation = axis-angle rotvec)
    (LeRobot aggregates these flat float keys into observation.state / action.)

NOTE: build the LeRobotDataset with YOUR installed lerobot's dataset API; its
schema / recording helpers differ across releases.
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
            # 7-vector [dx, dy, dz, drx, dry, drz, gripper] in [-1, 1];
            # here we send a small +x nudge with the gripper open.
            action = np.array([0.3, 0, 0, 0, 0, 0, 1.0], dtype=np.float32)
            adapter.send_action(action)
            frames.append((obs, action))

        print(f"collected {len(frames)} frames")
        try:
            import lerobot  # noqa: F401

            print("lerobot is installed. This example does NOT build a dataset; "
                  "to persist `frames`, create a LeRobotDataset and add_frame() "
                  "each (obs, action) with a per-frame task string, using the "
                  "dataset API of YOUR lerobot version (it differs across releases).")
        except Exception:
            print("(lerobot not installed -> frames printed only; install "
                  "flexiv-control[lerobot] to write a real dataset.)")
            print("  example frame 0 obs keys:", list(frames[0][0].keys()))
    finally:
        adapter.disconnect()


if __name__ == "__main__":
    main()
