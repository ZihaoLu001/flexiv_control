"""The unified *action contract*.

This is the single most important module for cross-project reuse. Every high
level component -- a receding-horizon planner, an MPC loop, an RL policy, a
SpaceMouse teleop bridge -- emits one of these objects, and every backend knows
how to execute them. Nobody re-invents "how do I talk to the robot".

Why this exact shape
--------------------
Action-chunking and receding-horizon planners typically emit a candidate action
of the form

    u = ( (x_j, y_j, z_j, w_j, n_j) )_{j=1..H}

i.e. a short sequence of Cartesian waypoints, each with a gripper command ``w_j``
and a *number of low-level control frames* ``n_j``. That ``n_j`` is precisely a
duration once you fix a control rate:

    duration_j = n_frames_j / control_hz

``CartesianChunk`` below is that object, generalised so it is also exactly what
an MPC horizon, a robosuite/MuJoCo rollout, or an RL action-chunk needs:
  * positions become full SE(3) poses (orientation may be *held* -> position-only
    chunks map in with zero changes),
  * per-waypoint stiffness/limits so the *same* chunk can describe a free-space
    reach and a contact-rich push,
  * an explicit safety_profile name so execution is reproducible.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional

import numpy as np

from .types import CART_DOF, ForceControlParams, GripperCommand, ImpedanceParams


# ----------------------------------------------------------------------------
# Cartesian
# ----------------------------------------------------------------------------
@dataclass
class CartesianWaypoint:
    """One Cartesian target.

    ``position`` is always required. ``quaternion`` (w, x, y, z) is optional:
    leave it ``None`` to *hold the current/previous orientation*, which is what
    a position-only planner (e.g. one emitting ``(x, y, z)`` waypoints) wants.

    Either ``n_frames`` (a per-waypoint frame count) or ``duration`` may be given. The
    interpolator converts ``n_frames`` to a duration using the active control
    rate, so the same chunk runs identically whether the loop is 100 Hz or
    1 kHz, as long as ``n_frames`` is interpreted at that rate.
    """

    position: np.ndarray
    quaternion: Optional[np.ndarray] = None   # (w, x, y, z); None -> hold
    gripper: Optional[GripperCommand] = None  # None -> hold gripper
    n_frames: Optional[int] = None            # number of low-level control frames
    duration: Optional[float] = None          # seconds (alternative to n_frames)
    frame: str = "base"

    def __post_init__(self) -> None:
        self.position = np.asarray(self.position, float).reshape(3)
        if self.quaternion is not None:
            q = np.asarray(self.quaternion, float).reshape(4)
            n = np.linalg.norm(q)
            if n < 1e-9:
                raise ValueError("zero-norm quaternion")
            self.quaternion = q / n
        if self.n_frames is None and self.duration is None:
            raise ValueError("CartesianWaypoint needs either n_frames or duration")
        if self.n_frames is not None and self.n_frames <= 0:
            raise ValueError("n_frames must be positive")

    def resolve_duration(self, control_hz: float) -> float:
        if self.duration is not None:
            return float(self.duration)
        return float(self.n_frames) / float(control_hz)


@dataclass
class CartesianChunk:
    """A short, bounded sequence of Cartesian waypoints -- the core action type.

    Used by a receding-horizon planner (execute first segment, replan), MPC (the first slice of a
    horizon), scripted manipulation, and high-level RL.
    """

    waypoints: List[CartesianWaypoint]

    # Compliance for the whole chunk (a waypoint may override later if needed).
    impedance: ImpedanceParams = field(default_factory=ImpedanceParams)
    force_control: Optional[ForceControlParams] = None

    # Kinematic envelope enforced by the interpolator + safety filter.
    max_tcp_linear_speed: float = 0.25    # m/s
    max_tcp_angular_speed: float = 0.60   # rad/s
    max_tcp_linear_acc: float = 1.0       # m/s^2
    max_tcp_angular_acc: float = 2.0      # rad/s^2

    # Contact envelope (None -> use the safety profile default).
    max_contact_wrench: Optional[np.ndarray] = None  # [fx,fy,fz,tx,ty,tz]

    # Named safety profile to load before executing (reproducibility).
    safety_profile: str = "tabletop_safe"

    frame: str = "base"

    def __post_init__(self) -> None:
        if not self.waypoints:
            raise ValueError("CartesianChunk needs at least one waypoint")
        if self.max_contact_wrench is not None:
            self.max_contact_wrench = np.asarray(self.max_contact_wrench, float).reshape(CART_DOF)

    @property
    def horizon(self) -> int:
        return len(self.waypoints)

    def total_duration(self, control_hz: float) -> float:
        return sum(w.resolve_duration(control_hz) for w in self.waypoints)

    # -- Convenience constructors -------------------------------------------
    @classmethod
    def from_waypoint_array(
        cls,
        u: np.ndarray,
        *,
        gripper_force: float = 20.0,
        hold_orientation: bool = True,
        **chunk_kwargs,
    ) -> "CartesianChunk":
        """Build a chunk directly from a planner's ``(H, 5)`` action array ``u``.

        ``u`` is an ``(H, 5)`` array of rows ``(x, y, z, w, n)`` where ``w`` is a
        normalised gripper command in ``[0, 1]`` (1 = open, 0 = closed) and ``n``
        is the integer number of low-level control frames. Orientation is held
        from the previous pose by default.
        """
        u = np.asarray(u, float)
        if u.ndim != 2 or u.shape[1] != 5:
            raise ValueError("u must have shape (H, 5): (x, y, z, w, n)")
        # An (H, 5) array carries no orientation, so every waypoint holds the
        # previous orientation (quaternion=None). ``hold_orientation`` is accepted
        # for API symmetry but is necessarily True for this position-only format;
        # pass full SE(3) ``CartesianWaypoint``s if you need to command orientation.
        if not hold_orientation:
            raise ValueError(
                "from_waypoint_array ingests position-only (H,5) actions and cannot "
                "set orientation; build CartesianWaypoint(s) with quaternions instead."
            )
        wpts: List[CartesianWaypoint] = []
        for x, y, z, w, n in u:
            grip = GripperCommand(width=float(np.clip(w, 0, 1)) * 0.08, force=gripper_force)
            wpts.append(
                CartesianWaypoint(
                    position=[x, y, z],
                    quaternion=None,
                    gripper=grip,
                    n_frames=max(1, int(round(n))),
                )
            )
        return cls(waypoints=wpts, **chunk_kwargs)


# ----------------------------------------------------------------------------
# Delta (relative) servo command -- the RL / MPC / teleop workhorse
# ----------------------------------------------------------------------------
@dataclass
class CartesianDelta:
    """A relative end-effector move, integrated on top of the current pose.

    This is the standard RL / MPC / teleop action: ``[dx, dy, dz, droll, dpitch,
    dyaw]`` plus a gripper command, applied over ``duration`` seconds. It maps
    1:1 onto robosuite/MuJoCo OSC-style actions, which is what makes sim->real
    transfer clean.
    """

    delta: np.ndarray                       # length-6, base or tcp frame
    gripper: Optional[GripperCommand] = None
    duration: float = 0.05                  # 20 Hz default control step
    frame: str = "base"

    def __post_init__(self) -> None:
        self.delta = np.asarray(self.delta, float).reshape(CART_DOF)


# ----------------------------------------------------------------------------
# Joint space (reset / home / MoveIt-plan execution)
# ----------------------------------------------------------------------------
@dataclass
class JointWaypoint:
    positions: np.ndarray
    n_frames: Optional[int] = None
    duration: Optional[float] = None

    def __post_init__(self) -> None:
        self.positions = np.asarray(self.positions, float)
        if self.n_frames is None and self.duration is None:
            raise ValueError("JointWaypoint needs n_frames or duration")

    def resolve_duration(self, control_hz: float) -> float:
        if self.duration is not None:
            return float(self.duration)
        return float(self.n_frames) / float(control_hz)


@dataclass
class JointChunk:
    waypoints: List[JointWaypoint]
    max_joint_speed_scale: float = 0.3   # fraction of joint vel limits
    safety_profile: str = "tabletop_safe"

    def __post_init__(self) -> None:
        if not self.waypoints:
            raise ValueError("JointChunk needs at least one waypoint")


# ----------------------------------------------------------------------------
# Execution report -- quantifies the "execution" failure category
# ----------------------------------------------------------------------------
@dataclass
class ExecutionResult:
    """Returned by every blocking execution call.

    A receding-horizon planner's failure taxonomy typically has an "execution"
    bucket. This object turns
    that bucket into something observable and quantifiable rather than a guess:
    the planner logs ``clipped``/``stop_reason``/``path_tracking_error`` and can
    attribute a bad outcome to execution vs perception/ranking with evidence.
    """

    success: bool = True
    clipped: bool = False
    stop_reason: str = "none"
    executed_duration: float = 0.0
    path_tracking_error: float = 0.0      # max ||pose_cmd - pose_meas|| over run
    max_tcp_speed: float = 0.0
    max_joint_speed: float = 0.0
    max_wrench: float = 0.0
    gripper_width_final: float = 0.0
    final_state: Optional["object"] = None  # RobotState; Optional to avoid import cycle
    log: dict = field(default_factory=dict)
