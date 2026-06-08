"""A Gymnasium environment over the unified controller.

The single most important thing for sim-to-real RL is that the *same action
contract* drives sim and real. This env exposes the standard low-dim Cartesian
action used by robosuite/SERL/LeRobot -- ``[dx, dy, dz, drx, dry, drz,
gripper]`` in ``[-1, 1]`` (the rotation triplet is an axis-angle rotation
vector, not Euler roll/pitch/yaw) -- and maps it through the same safety filter,
interpolation, and backend as everything else. Point it at the ``fake`` backend
to develop offline, the ``mujoco`` backend for sim, or a real Rizon (directly or
via :class:`~flexiv_control.RemoteRobot`) with **no change to the policy**.

Gymnasium is optional: if it is not installed we fall back to a tiny ``Env`` /
``Box`` shim so the class still imports and runs (handy for CI). Install
``gymnasium`` for the real thing (``pip install "flexiv-control[rl]"``).
"""

from __future__ import annotations

from typing import Any, Callable, Dict, Optional, Tuple

import numpy as np

from ..config import RobotConfig
from ..robot import Robot
from ..types import GripperCommand

# -- gymnasium-optional shim -------------------------------------------------
try:  # pragma: no cover - exercised only when gymnasium is installed
    import gymnasium as gym
    from gymnasium import spaces

    _EnvBase = gym.Env
    HAVE_GYM = True
except Exception:  # minimal stand-ins so the env imports/runs without gymnasium

    class _Box:  # noqa: D401 - tiny shim
        def __init__(self, low, high, shape=None, dtype=np.float32):
            self.low = np.asarray(low, dtype) if np.ndim(low) else np.full(shape, low, dtype)
            self.high = np.asarray(high, dtype) if np.ndim(high) else np.full(shape, high, dtype)
            self.shape = self.low.shape
            self.dtype = dtype

        def sample(self) -> np.ndarray:
            return np.random.uniform(self.low, self.high).astype(self.dtype)

        def contains(self, x) -> bool:
            x = np.asarray(x)
            return bool(np.all(x >= self.low) and np.all(x <= self.high))

    class _Spaces:
        Box = _Box

    class _EnvBase:  # mimics gym.Env surface we use
        metadata: Dict[str, Any] = {}

        def reset(self, *args, **kwargs):
            raise NotImplementedError

        def step(self, action):
            raise NotImplementedError

        def close(self):
            pass

    spaces = _Spaces()  # type: ignore
    HAVE_GYM = False


RewardFn = Callable[["np.ndarray", "np.ndarray"], Tuple[float, bool, dict]]


class FlexivRealEnv(_EnvBase):
    """Cartesian-delta Gymnasium env on the unified controller."""

    metadata = {"render_modes": []}

    def __init__(
        self,
        robot: Optional[object] = None,
        *,
        config: Optional[RobotConfig] = None,
        control_hz: float = 20.0,
        action_horizon: int = 1,
        max_episode_steps: int = 200,
        safety_profile: str = "rl_conservative",
        pos_scale: float = 0.05,
        rot_scale: float = 0.20,
        gripper_open_width: float = 0.08,
        gripper_force: float = 20.0,
        home_pose: Optional[np.ndarray] = None,
        owner: str = "rl_env",
        reward_fn: Optional[RewardFn] = None,
    ):
        self.robot = robot or Robot(config or RobotConfig(control_hz=control_hz))
        self.control_hz = float(control_hz)
        self.action_horizon = int(action_horizon)
        self.step_duration = self.action_horizon / self.control_hz
        self.max_episode_steps = int(max_episode_steps)
        self.safety_profile = safety_profile
        self.pos_scale = float(pos_scale)
        self.rot_scale = float(rot_scale)
        self.gripper_open_width = float(gripper_open_width)
        self.gripper_force = float(gripper_force)
        self.home_pose = None if home_pose is None else np.asarray(home_pose, float).reshape(7)
        self.owner = owner
        self.reward_fn = reward_fn

        self._connected = False
        self._elapsed = 0

        # action: [dx, dy, dz, drx, dry, drz, gripper] in [-1, 1]
        # (drx/dry/drz = axis-angle rotation-vector delta, not Euler)
        self.action_space = spaces.Box(-1.0, 1.0, shape=(7,), dtype=np.float32)
        # observation: [q(7), dq(7), tcp_pose(7), wrench(6), gripper_width(1)]
        self._obs_dim = 7 + 7 + 7 + 6 + 1
        high = np.full(self._obs_dim, np.inf, np.float32)
        self.observation_space = spaces.Box(-high, high, dtype=np.float32)

    # -- helpers -------------------------------------------------------------
    def _ensure_connected(self) -> None:
        if self._connected:
            return
        self.robot.connect()
        try:
            self.robot.acquire_lease(self.owner)  # in-process Robot
        except TypeError:
            self.robot.acquire_lease()  # RemoteRobot
        try:
            self.robot.set_safety_profile(self.safety_profile)
        except FileNotFoundError:
            pass  # profile not shipped in this checkout; keep default
        self._connected = True

    def _obs(self) -> np.ndarray:
        s = self.robot.get_state()
        return np.concatenate(
            [s.q, s.dq, s.tcp_pose, s.wrench, [s.gripper_width]]
        ).astype(np.float32)

    def _scale_action(self, action: np.ndarray) -> Tuple[np.ndarray, GripperCommand]:
        a = np.clip(np.asarray(action, float).reshape(7), -1.0, 1.0)
        delta = np.empty(6)
        delta[:3] = a[:3] * self.pos_scale
        delta[3:] = a[3:6] * self.rot_scale
        # gripper: shared [-1,1] encoding (+1 open, -1 closed + grasp).
        grip = GripperCommand.from_signed_action(
            a[6], span=self.gripper_open_width, force=self.gripper_force
        )
        return delta, grip

    # -- gym API -------------------------------------------------------------
    def reset(self, *, seed: Optional[int] = None, options: Optional[dict] = None):
        if HAVE_GYM:
            # Seed self.np_random + run gym's reset bookkeeping (the Gymnasium
            # contract). Do NOT touch the global np.random RNG.
            super().reset(seed=seed)
        self._ensure_connected()
        # reset() must END in the Cartesian motion-force mode that step() needs.
        # robot.home() runs the NRT 'Home' primitive and leaves the real backend
        # in NRT_PRIMITIVE, so homing must come BEFORE entering impedance -- the
        # old order (impedance then home) left the arm in primitive mode and the
        # first step()'s Cartesian command would be rejected on hardware.
        if self.home_pose is not None:
            self.robot.start_cartesian_impedance()
            self.robot.servo_cartesian_pose(self.home_pose, duration=2.0)
        else:
            self.robot.home()
            self.robot.start_cartesian_impedance()
        self._elapsed = 0
        return self._obs(), {}

    def step(self, action: np.ndarray):
        self._elapsed += 1
        delta, grip = self._scale_action(action)
        result = self.robot.servo_cartesian_delta(
            delta, duration=self.step_duration, gripper=grip
        )
        obs = self._obs()

        if self.reward_fn is not None:
            reward, terminated, extra = self.reward_fn(obs, np.asarray(action, float))
        else:
            reward, terminated, extra = 0.0, False, {}

        truncated = self._elapsed >= self.max_episode_steps
        info = {
            "execution_success": result.success,
            "clipped": result.clipped,
            "stop_reason": result.stop_reason,
            "path_tracking_error": result.path_tracking_error,
            "max_wrench": result.max_wrench,
            **extra,
        }
        # A safety stop ends the episode.
        if not result.success:
            terminated = True
        return obs, float(reward), bool(terminated), bool(truncated), info

    def close(self) -> None:
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


def make_env(**kwargs) -> FlexivRealEnv:
    """Convenience factory. The env is also registered as ``FlexivReal-v0`` when
    gymnasium is installed, so ``gym.make("FlexivReal-v0", ...)`` works too."""
    return FlexivRealEnv(**kwargs)


# -- HIL-SERL-style human intervention wrapper ------------------------------
if HAVE_GYM:

    class SpacemouseIntervention(gym.Wrapper):
        """Human-in-the-loop intervention wrapper (SERL / HIL-SERL contract).

        Wrap a :class:`FlexivRealEnv` and pass any object exposing
        ``intervention(action) -> (action, intervened)`` -- e.g.
        :class:`flexiv_control.teleop.SpaceMouseTeleop`. When the operator takes
        over (deadman held + device moved) the human delta replaces the policy
        action; the executed action and a flag are written into ``info``
        (``info["intervene_action"]``, ``info["intervened"]``) so an actor-learner
        trains on the human correction, matching SERL's ``SpacemouseIntervention``.
        """

        def __init__(self, env, teleop):
            super().__init__(env)
            self.teleop = teleop

        def step(self, action):
            action, intervened = self.teleop.intervention(action)
            obs, reward, terminated, truncated, info = self.env.step(action)
            info["intervene_action"] = np.asarray(action, np.float32)
            info["intervened"] = bool(intervened)
            return obs, reward, terminated, truncated, info

    try:
        gym.register(id="FlexivReal-v0", entry_point="flexiv_control.envs:FlexivRealEnv")
    except Exception:  # already registered, or a gym build without register
        pass

else:  # pragma: no cover - needs gymnasium

    class SpacemouseIntervention:  # type: ignore[no-redef]
        def __init__(self, *a, **k):
            raise ImportError(
                "SpacemouseIntervention needs gymnasium: pip install 'flexiv-control[rl]'"
            )
