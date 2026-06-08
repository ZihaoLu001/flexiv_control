"""A LeRobot ``Robot``-interface adapter for the Flexiv controller.

LeRobot (HuggingFace) is the de-facto community hub for robot-learning data and
policies. Conforming to its "Bring Your Own Hardware" ``Robot`` interface lets a
Flexiv arm plug into LeRobot's data-collection, training, and visualization
tooling. This adapter exposes the LeRobot ``Robot`` surface; turning the recorded
frames into a ``LeRobotDataset`` (MP4 + Parquet) is a thin extra step on the
caller's side against their installed ``lerobot`` version (the dataset API moves
between releases -- see ``examples/06_lerobot_record.py``).

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

    # -- LeRobot feature schema ---------------------------------------------
    # LeRobot's modern Robot interface wants FLAT, bare per-scalar keys whose
    # value is the Python type ``float`` (cf. SOFollower's ``{f"{m}.pos": float}``).
    # LeRobot itself adds the ``observation.``/``action.`` prefix and aggregates
    # the float keys into a single ``observation.state`` / ``action`` vector
    # (names = the bare keys) via hw_to_dataset_features. get_observation /
    # send_action below return one scalar per key so build_dataset_frame can
    # index each by name.
    _OBS_KEYS = (
        [f"q.{i}" for i in range(7)]
        + [f"dq.{i}" for i in range(7)]
        + [f"tcp_pose.{i}" for i in range(7)]
        + [f"wrench.{i}" for i in range(6)]
        + ["gripper_width"]
    )
    # Rotation deltas are an AXIS-ANGLE rotation vector (consumed as a rotvec by
    # servo_cartesian_delta), NOT Euler roll/pitch/yaw -- name them honestly so
    # the recorded LeRobot action features can't be misread as Euler angles.
    _ACTION_KEYS = ("dx", "dy", "dz", "drx", "dry", "drz", "gripper")

    @property
    def observation_features(self) -> Dict[str, type]:
        return {k: float for k in self._OBS_KEYS}

    @property
    def action_features(self) -> Dict[str, type]:
        return {k: float for k in self._ACTION_KEYS}

    @property
    def is_connected(self) -> bool:
        return self._connected

    @property
    def is_calibrated(self) -> bool:
        # Rizon is factory-calibrated; calibrate() is a no-op, so report True to
        # let LeRobot's connect() skip self.calibrate().
        return True

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
    def get_observation(self) -> Dict[str, float]:
        # One scalar per bare key, matching observation_features, so LeRobot's
        # build_dataset_frame can index each by name into observation.state.
        s = self.robot.get_state()
        obs: Dict[str, float] = {}
        for i in range(7):
            obs[f"q.{i}"] = float(s.q[i])
        for i in range(7):
            obs[f"dq.{i}"] = float(s.dq[i])
        for i in range(7):
            obs[f"tcp_pose.{i}"] = float(s.tcp_pose[i])
        for i in range(6):
            obs[f"wrench.{i}"] = float(s.wrench[i])
        obs["gripper_width"] = float(s.gripper_width)
        return obs

    def send_action(
        self, action: Union[np.ndarray, Dict[str, float]]
    ) -> Dict[str, float]:
        # Accept a 7-vector or a per-key dict ({"dx":.., .., "gripper":..}); return
        # one scalar per action key so the recorded action frame matches features.
        if isinstance(action, dict):
            a = np.asarray([action[k] for k in self._ACTION_KEYS], float)
        else:
            a = np.asarray(action, float).reshape(7)
        a = np.clip(a, -1.0, 1.0)
        delta = np.empty(6)
        delta[:3] = a[:3] * self.pos_scale
        delta[3:] = a[3:6] * self.rot_scale
        grip = GripperCommand.from_signed_action(a[6], span=self.gripper_open_width, force=20.0)
        self.robot.servo_cartesian_delta(delta, duration=self.step_duration, gripper=grip)
        return {k: float(v) for k, v in zip(self._ACTION_KEYS, a)}


def register_with_lerobot() -> bool:
    """Attempt to register the adapter with an installed LeRobot.

    NOT IMPLEMENTED: current LeRobot has no stable runtime "register a robot"
    call -- hardware is registered via a config dataclass decorated with
    ``@RobotConfig.register_subclass("flexiv_rizon")`` plus the
    ``make_robot_from_config`` factory, so there is nothing to invoke at runtime
    here. Use :class:`LeRobotFlexivAdapter` directly; it implements the full
    LeRobot ``Robot`` surface. Returns ``False`` (no registration performed).
    """
    return False
