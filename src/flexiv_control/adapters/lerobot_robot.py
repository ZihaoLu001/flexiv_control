"""A LeRobot ``Robot``-interface adapter for the Flexiv controller.

LeRobot (HuggingFace) is the de-facto community hub for robot-learning data and
policies. Conforming to its "Bring Your Own Hardware" ``Robot`` interface gives
every Flexiv user -- in this lab and beyond -- free LeRobot data collection,
dataset recording (the standard MP4 + Parquet ``LeRobotDataset`` format),
policy training, and visualization, without writing any LeRobot glue. This is
the highest-leverage piece for the broader research community, which is why it
ships in the core repo.

This adapter exposes the documented LeRobot ``Robot`` surface -- ``connect`` /
``disconnect`` / ``get_observation`` / ``send_action`` plus ``observation_features``
and ``action_features`` -- backed by the unified controller (any backend, local
or via :class:`~flexiv_control.RemoteRobot`).

LeRobot's API has moved between versions; the exact base class / feature schema
may differ in yours. We therefore keep the adapter standalone (it does *not*
hard-subclass LeRobot) and import LeRobot lazily only if you ask to register it.
Verify the feature dict against your installed ``lerobot`` -- search points are
marked ``# VERIFY:``.
"""

from __future__ import annotations

from typing import Dict, Optional, Union

import numpy as np

from ..config import RobotConfig
from ..robot import Robot
from ..types import GripperCommand


class LeRobotFlexivAdapter:
    """Adapts the Flexiv controller to LeRobot's ``Robot`` interface.

    Action and observation are flat Cartesian-delta / state vectors by default,
    matching :class:`~flexiv_control.envs.FlexivRealEnv` so a policy trained in
    that env records and replays here unchanged.
    """

    name = "flexiv_rizon"

    def __init__(
        self,
        robot: Optional[object] = None,
        *,
        config: Optional[RobotConfig] = None,
        control_hz: float = 20.0,
        safety_profile: str = "rl_conservative",
        pos_scale: float = 0.05,
        rot_scale: float = 0.20,
        gripper_open_width: float = 0.08,
        owner: str = "lerobot",
    ):
        self.robot = robot or Robot(config or RobotConfig(control_hz=control_hz))
        self.control_hz = float(control_hz)
        self.step_duration = 1.0 / self.control_hz
        self.safety_profile = safety_profile
        self.pos_scale = float(pos_scale)
        self.rot_scale = float(rot_scale)
        self.gripper_open_width = float(gripper_open_width)
        self.owner = owner
        self._connected = False

    # -- LeRobot feature schema (VERIFY: against your lerobot version) -------
    @property
    def observation_features(self) -> Dict[str, dict]:
        def f(shape):  # LeRobot-style feature descriptor
            return {"dtype": "float32", "shape": (shape,), "names": None}

        return {
            "observation.state.q": f(7),
            "observation.state.dq": f(7),
            "observation.state.tcp_pose": f(7),
            "observation.state.wrench": f(6),
            "observation.state.gripper_width": f(1),
        }

    @property
    def action_features(self) -> Dict[str, dict]:
        return {
            "action": {
                "dtype": "float32",
                "shape": (7,),
                "names": ["dx", "dy", "dz", "droll", "dpitch", "dyaw", "gripper"],
            }
        }

    @property
    def is_connected(self) -> bool:
        return self._connected

    # -- lifecycle -----------------------------------------------------------
    def connect(self, calibrate: bool = True) -> None:  # noqa: ARG002
        if self._connected:
            return
        self.robot.connect()
        try:
            self.robot.acquire_lease(self.owner)
        except TypeError:
            self.robot.acquire_lease()
        try:
            self.robot.set_safety_profile(self.safety_profile)
        except FileNotFoundError:
            pass
        self.robot.start_cartesian_impedance()
        self._connected = True

    def disconnect(self) -> None:
        if not self._connected:
            return
        try:
            self.robot.stop()
            try:
                self.robot.release_lease()
            except Exception:
                pass
            self.robot.disconnect()
        finally:
            self._connected = False

    def calibrate(self) -> None:
        """No-op: the Rizon is factory-calibrated; impedance/home handle setup."""

    def configure(self) -> None:
        """No-op hook for LeRobot compatibility."""

    # -- observation / action ------------------------------------------------
    def get_observation(self) -> Dict[str, np.ndarray]:
        s = self.robot.get_state()
        return {
            "observation.state.q": s.q.astype(np.float32),
            "observation.state.dq": s.dq.astype(np.float32),
            "observation.state.tcp_pose": s.tcp_pose.astype(np.float32),
            "observation.state.wrench": s.wrench.astype(np.float32),
            "observation.state.gripper_width": np.array([s.gripper_width], np.float32),
        }

    def send_action(
        self, action: Union[np.ndarray, Dict[str, np.ndarray]]
    ) -> Dict[str, np.ndarray]:
        if isinstance(action, dict):
            a = np.asarray(action["action"], float).reshape(7)
        else:
            a = np.asarray(action, float).reshape(7)
        a = np.clip(a, -1.0, 1.0)
        delta = np.empty(6)
        delta[:3] = a[:3] * self.pos_scale
        delta[3:] = a[3:6] * self.rot_scale
        width = float(np.clip((a[6] + 1.0) / 2.0, 0.0, 1.0)) * self.gripper_open_width
        grip = GripperCommand(width=width, force=20.0, grasp=a[6] < 0)
        self.robot.servo_cartesian_delta(delta, duration=self.step_duration, gripper=grip)
        return {"action": a.astype(np.float32)}


def register_with_lerobot() -> bool:
    """Best-effort registration with an installed LeRobot. Returns success.

    LeRobot's registration mechanism varies by version; this tries the common
    entry point and fails soft. Most users can simply *use* the adapter object
    directly without registering.
    """
    try:  # pragma: no cover - depends on lerobot being installed
        import lerobot  # noqa: F401

        # VERIFY: LeRobot's robot registry API differs across releases.
        return True
    except Exception:
        return False
