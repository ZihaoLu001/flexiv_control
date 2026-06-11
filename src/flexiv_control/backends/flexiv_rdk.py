"""Flexiv RDK backend -- the real Rizon arm.

This wraps Flexiv RDK (Python module ``flexivrdk``). It supports both RT modes
(``Stream*`` at up to 1 kHz; the robot runs the hard real-time motion-force /
impedance loop internally) and NRT modes (``Send*``; the robot's internal motion
generator interpolates discrete commands, tolerant of a non-RT host).

IMPORTANT, please read before running on hardware
--------------------------------------------------
* RT streaming requires the **Professional** RDK license; NRT works on Standard.
* Enable "Remote/RDK" mode on the robot via Flexiv Elements first.
* RDK attribute/enum names have changed across versions (e.g. the v1.x renames
  to math-symbol members). Every spot that depends on a specific name is marked
  ``# VERIFY:``. Run ``tests/hardware_smoke.py`` and the read-only example
  before commanding motion, and fix any name that differs on your RDK version.
* The guarded import means importing this module never fails; the clear error is
  raised only if you actually try to ``connect()`` without ``flexivrdk``.
"""

from __future__ import annotations

import time
from typing import Any, Optional

import numpy as np

from ..types import (
    CART_DOF,
    ControlMode,
    ForceControlParams,
    GripperCommand,
    ImpedanceParams,
    JointImpedanceParams,
    RobotState,
    SafetyStatus,
    StopReason,
)
from .base import RobotBackend

try:  # guarded import: never crash on machines without RDK installed
    import flexivrdk  # type: ignore

    _RDK_AVAILABLE = True
    _RDK_IMPORT_ERROR: Optional[Exception] = None
except Exception as exc:  # pragma: no cover - depends on environment
    flexivrdk = None  # type: ignore
    _RDK_AVAILABLE = False
    _RDK_IMPORT_ERROR = exc


# Map our ControlMode -> flexivrdk.Mode. Built lazily because the enum only
# exists once flexivrdk is importable. # VERIFY enum member names per RDK version.
def _rdk_mode(mode: ControlMode):
    M = flexivrdk.Mode
    table = {
        ControlMode.IDLE: M.IDLE,
        ControlMode.NRT_JOINT_POSITION: M.NRT_JOINT_POSITION,
        ControlMode.NRT_JOINT_IMPEDANCE: M.NRT_JOINT_IMPEDANCE,
        ControlMode.NRT_CARTESIAN_MOTION_FORCE: M.NRT_CARTESIAN_MOTION_FORCE,
        ControlMode.NRT_PRIMITIVE: M.NRT_PRIMITIVE_EXECUTION,
        ControlMode.RT_JOINT_POSITION: M.RT_JOINT_POSITION,
        ControlMode.RT_JOINT_IMPEDANCE: M.RT_JOINT_IMPEDANCE,
        ControlMode.RT_CARTESIAN_MOTION_FORCE: M.RT_CARTESIAN_MOTION_FORCE,
        ControlMode.RT_JOINT_TORQUE: M.RT_JOINT_TORQUE,
    }
    return table[mode]


def _get(obj: Any, *names: str, default=None):
    """Return the first present attribute from ``names`` (cross-version safe)."""
    for n in names:
        if hasattr(obj, n):
            return getattr(obj, n)
    return default


def _rdk_coord(frame: str):
    """Map a frame name to ``flexivrdk.CoordType``.

    RDK's ``SetForceControlFrame`` takes a ``CoordType`` enum (``WORLD``/``TCP``),
    *not* a string. "tcp"/"flange"/"ee" -> TCP; everything else ("base"/"world")
    -> WORLD. Confirmed against RDK v1.x ``robot.hpp``.
    """
    C = flexivrdk.CoordType
    return C.TCP if str(frame).lower() in ("tcp", "flange", "ee") else C.WORLD


def _warn_rdk_version() -> None:
    """Warn if the installed flexivrdk is outside the supported v1.x range.

    Enforces (as a soft check) the version discipline docs/versions.md documents:
    this backend targets RDK v1.x; v0.x and v2.x have incompatible APIs.
    """
    v = getattr(flexivrdk, "__version__", None)
    if not v:
        return
    try:
        major = int(str(v).split(".")[0])
    except Exception:
        return
    if major != 1:
        import warnings

        warnings.warn(
            f"flexivrdk {v} detected, but this backend targets RDK v1.x. "
            "v0.x and v2.x have incompatible APIs (constructor, state fields, "
            "command form); see docs/versions.md. Pin flexivrdk>=1.5,<2.",
            RuntimeWarning,
            stacklevel=2,
        )


class FlexivRdkBackend(RobotBackend):
    def __init__(
        self,
        robot_sn: str,
        n_joints: int = 7,
        gripper_name: Optional[str] = None,
        *,
        allow_torque: bool = False,
    ):
        if not _RDK_AVAILABLE:
            raise ImportError(
                "flexivrdk is not installed. Install with "
                "`pip install flexivrdk` (match the version to your robot "
                f"software). Original import error: {_RDK_IMPORT_ERROR}"
            )
        self.robot_sn = robot_sn
        self.n_joints = n_joints
        self._gripper_name = gripper_name
        self._allow_torque = bool(allow_torque)
        self._robot = None
        self._gripper = None
        self._mode = ControlMode.IDLE
        self._connected = False
        # The active Cartesian force-control params, kept so stream_cartesian can
        # actually command the configured target_wrench (not just a zero default).
        self._force_control: Optional[ForceControlParams] = None

    # -- lifecycle ----------------------------------------------------------
    def connect(self) -> None:
        _warn_rdk_version()
        self._robot = flexivrdk.Robot(self.robot_sn)  # VERIFY: ctor signature

        # Clear any minor fault, then enable + wait until operational.
        fault_fn = _get(self._robot, "fault", default=None)
        if callable(fault_fn) and fault_fn():
            ok = self._robot.ClearFault()  # VERIFY: v1.x returns bool, older None
            if ok is False:
                raise RuntimeError(
                    "could not clear robot fault -- check the pendant; an engaged "
                    "E-stop or active fault must be cleared before Enable()."
                )
        self._robot.Enable()
        t0 = time.time()
        while not self._robot.operational():  # VERIFY: operational() vs Operational
            if time.time() - t0 > 10.0:
                raise TimeoutError("robot did not become operational within 10 s")
            time.sleep(0.05)

        # Adopt the true DoF from the robot rather than trusting the constructor
        # default, and assert any explicit n_joints matches (a mismatch would
        # silently mis-size every command/state vector).
        try:
            qv = _get(self._robot.states(), "q", default=None)
            n = len(list(qv)) if qv is not None else 0
            if n:
                if self.n_joints and self.n_joints != n:
                    raise ValueError(
                        f"configured n_joints={self.n_joints} but the robot reports "
                        f"{n} joints; fix the config."
                    )
                self.n_joints = n
        except ValueError:
            raise
        except Exception:
            pass

        if self._gripper_name is not None:
            self._gripper = flexivrdk.Gripper(self._robot)  # VERIFY
            # RDK v1.x: Gripper.Init() (blocking, no args). Older APIs used
            # Enable(name). Try the v1.x form first, fall back, and SURFACE a
            # failure (a warning) instead of silently swallowing it.
            try:
                if hasattr(self._gripper, "Init"):
                    self._gripper.Init()
                elif hasattr(self._gripper, "Enable"):
                    self._gripper.Enable(self._gripper_name)
            except Exception as e:  # pragma: no cover - hardware-only path
                import warnings

                warnings.warn(
                    f"gripper init failed ({e}); gripper commands will be no-ops.",
                    RuntimeWarning,
                    stacklevel=2,
                )
                self._gripper = None
        self._connected = True

    def disconnect(self) -> None:
        if self._robot is not None:
            try:
                self._robot.Stop()
            except Exception:
                pass
        self._connected = False

    @property
    def is_connected(self) -> bool:
        return self._connected

    # -- state --------------------------------------------------------------
    def read_state(self) -> RobotState:
        s = self._robot.states()  # VERIFY: states() returns RobotStates
        q = np.asarray(_get(s, "q", default=np.zeros(self.n_joints)), float)
        dq = np.asarray(_get(s, "dq", "dtheta", default=np.zeros(self.n_joints)), float)
        tau = np.asarray(_get(s, "tau", default=np.zeros(self.n_joints)), float)
        tcp_pose = np.asarray(_get(s, "tcp_pose", "tcpPose", default=[0, 0, 0, 1, 0, 0, 0]), float)
        tcp_vel = np.asarray(_get(s, "tcp_vel", "tcpVel", default=np.zeros(CART_DOF)), float)
        wrench = np.asarray(
            _get(s, "ext_wrench_in_tcp", "ext_wrench_in_world", "extWrenchInTcp",
                 default=np.zeros(CART_DOF)),
            float,
        )

        gw, gf, gm = 0.0, 0.0, False
        if self._gripper is not None:
            try:
                gs = self._gripper.states()  # VERIFY
                gw = float(_get(gs, "width", default=0.0))
                gf = float(_get(gs, "force", default=0.0))
                # v1.x exposes motion via the Gripper.moving() METHOD, not a
                # states() field (GripperStates has no is_moving in v1.x, so the
                # old field read silently always returned False).
                if hasattr(self._gripper, "moving"):
                    gm = bool(self._gripper.moving())
                else:
                    gm = bool(_get(gs, "is_moving", "isMoving", default=False))
            except Exception:
                pass

        # Surface a robot-side fault through the state so the control loop (and
        # any client polling get_state) sees it -- previously hardcoded OK.
        faulted = self.in_fault()
        return RobotState(
            stamp=time.time(),
            q=q, dq=dq, tau=tau,
            tcp_pose=tcp_pose, tcp_vel=tcp_vel, wrench=wrench,
            gripper_width=gw, gripper_force=gf, gripper_is_moving=gm,
            control_mode=self._mode,
            safety_status=SafetyStatus.FAULT if faulted else SafetyStatus.OK,
            stop_reason=StopReason.BACKEND_FAULT if faulted else StopReason.NONE,
        )

    def in_fault(self) -> bool:
        """True if the robot reports a fault / protective stop (RDK ``fault()``)."""
        fault_fn = _get(self._robot, "fault", default=None)
        try:
            return bool(fault_fn()) if callable(fault_fn) else False
        except Exception:
            return False

    # -- mode ---------------------------------------------------------------
    def set_mode(
        self,
        mode: ControlMode,
        *,
        impedance: Optional[ImpedanceParams] = None,
        joint_impedance: Optional[JointImpedanceParams] = None,
        force_control: Optional[ForceControlParams] = None,
        nullspace_q: Optional[np.ndarray] = None,
        max_contact_wrench: Optional[np.ndarray] = None,
    ) -> None:
        if mode == ControlMode.RT_JOINT_TORQUE and not self._allow_torque:
            # Active gate (not just gating-by-omission): direct joint torque
            # bypasses the robot's impedance safety loop, so it is opt-in only.
            raise RuntimeError(
                "RT_JOINT_TORQUE is gated off. Construct "
                "FlexivRdkBackend(..., allow_torque=True) to enable direct "
                "joint-torque control (expert/research mode)."
            )
        self._robot.SwitchMode(_rdk_mode(mode))  # VERIFY
        self._mode = mode

        if impedance is not None and mode.is_cartesian:
            self._robot.SetCartesianImpedance(  # VERIFY arg order/name
                list(impedance.stiffness), list(impedance.damping_ratio)
            )
        if joint_impedance is not None and not mode.is_cartesian:
            self._robot.SetJointImpedance(
                list(joint_impedance.stiffness), list(joint_impedance.damping_ratio)
            )
        if max_contact_wrench is not None and mode.is_cartesian:
            self._robot.SetMaxContactWrench(list(np.asarray(max_contact_wrench, float)))
        if force_control is not None and mode.is_cartesian:
            self._force_control = force_control
            self._robot.SetForceControlAxis(list(map(bool, force_control.enabled_axes)))
            # SetForceControlFrame takes a CoordType enum, not a string. (RDK v1.x)
            self._robot.SetForceControlFrame(_rdk_coord(force_control.frame))
        elif not mode.is_cartesian:
            self._force_control = None  # force control is scoped to Cartesian modes
        if nullspace_q is not None and mode.is_cartesian:
            self._robot.SetNullSpacePosture(list(np.asarray(nullspace_q, float)))

    # -- streaming ----------------------------------------------------------
    def stream_cartesian(self, pose: np.ndarray, wrench: Optional[np.ndarray] = None) -> None:
        pose = list(np.asarray(pose, float).reshape(7))
        # RDK v1.x Stream/SendCartesianMotionForce take wrench[6] as the 2nd arg;
        # it only acts on axes enabled via SetForceControlAxis, so a zero default
        # is safe for pure-motion ticks. Dropping it (pose-only) silently disabled
        # force control during chunk execution. Precedence: an explicit per-tick
        # wrench wins; otherwise command the active mode's configured
        # target_wrench (so ForceControlParams.target_wrench is actually applied);
        # otherwise zero.
        if wrench is not None:
            wr = list(np.asarray(wrench, float).reshape(CART_DOF))
        elif self._force_control is not None:
            wr = list(np.asarray(self._force_control.target_wrench, float).reshape(CART_DOF))
        else:
            wr = [0.0] * CART_DOF
        if self._mode.is_realtime:
            # RT: 1 kHz streaming, robot tracks immediately.
            self._robot.StreamCartesianMotionForce(pose, wr)
        else:
            # NRT: discrete target, robot's motion generator interpolates.
            self._robot.SendCartesianMotionForce(pose, wr)

    def stream_joint(self, q: np.ndarray) -> None:
        q = list(np.asarray(q, float))
        zeros = [0.0] * self.n_joints
        if self._mode.is_realtime:
            # RT: StreamJointPosition(positions, velocities, accelerations). (RDK v1.x)
            self._robot.StreamJointPosition(q, zeros, zeros)
        else:
            # NRT: SendJointPosition(positions, velocities, max_vel, max_acc) -- the
            # 3rd/4th args are trajectory *limits*, not a target acceleration. The
            # earlier 5-arg form (extra acc vector) is wrong for RDK v1.x.
            max_vel = [1.0] * self.n_joints
            max_acc = [1.0] * self.n_joints
            self._robot.SendJointPosition(q, zeros, max_vel, max_acc)

    # -- gripper ------------------------------------------------------------
    def move_gripper(self, cmd: GripperCommand) -> None:
        if self._gripper is None:
            return
        if cmd.grasp:
            self._gripper.Grasp(cmd.force)  # VERIFY
        else:
            self._gripper.Move(cmd.width, cmd.velocity, cmd.force)  # VERIFY

    # -- safety -------------------------------------------------------------
    def stop(self) -> None:
        self._robot.Stop()
        self._mode = ControlMode.IDLE

    def home(self, q_home: Optional[np.ndarray] = None) -> None:
        """Home via the NRT 'Home' primitive (robot-internal, safe).

        The vendor primitive goes to the FACTORY home, not a configured posture,
        so a non-None ``q_home`` raises ``NotImplementedError`` and lets
        ``Robot.home()`` fall back to a speed-capped interpolated joint move to
        the configured posture -- previously the argument was silently ignored
        and the arm ended somewhere other than the lab's canonical home.
        """
        if q_home is not None:
            raise NotImplementedError(
                "the RDK 'Home' primitive ignores a configured q_home; "
                "Robot.home falls back to an interpolated joint move"
            )
        self._robot.SwitchMode(_rdk_mode(ControlMode.NRT_PRIMITIVE))
        # ExecutePrimitive(name, input_params, block_until_started=True). (RDK v1.x)
        self._robot.ExecutePrimitive("Home", dict())
        # Block until the primitive reports completion. RDK v1.x exposes
        # primitive_states() as a dict; "reachedTarget" flips to "1" when done.
        # Fall back to busy() if that key is absent on this RDK version.
        t0 = time.time()
        while time.time() - t0 < 30.0:
            if not self._primitive_running():
                break
            time.sleep(0.05)
        self._mode = ControlMode.NRT_PRIMITIVE

    def _primitive_running(self) -> bool:
        """True while an NRT primitive is still executing (best-effort, cross-version)."""
        try:
            ps = self._robot.primitive_states()
            if isinstance(ps, dict) and "reachedTarget" in ps:
                return str(ps["reachedTarget"]).strip().lower() not in ("1", "true")
        except Exception:
            pass
        try:
            return bool(self._robot.busy())
        except Exception:
            return False
