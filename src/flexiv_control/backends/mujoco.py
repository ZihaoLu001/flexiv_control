"""MuJoCo backend (stub) for sim and real2sim2real.

The *intended* property (once this backend is implemented): the *same*
``CartesianChunk`` / ``CartesianDelta`` action and the *same* ``RobotState``
observation drive sim and the real robot, so real2sim2real and sim-trained
policies transfer with no rewrite -- the property Deoxys (robosuite->real) and
Polymetis (pybullet->real) are valued for. The interface/contract already
delivers this for the ``fake`` <-> ``flexiv_rdk`` pair.

NOTE: this MuJoCo backend is currently a **deliberate, non-functional stub** --
``connect()`` raises ``NotImplementedError`` rather than faking success. It
defines the structure and integration points but leaves the model load +
IK / operational-space mapping for you to fill in against your Rizon MJCF (the
Flexiv ``flexiv_description`` URDF / your MuJoCo model + a Jacobian or ``mujoco``
MJX IK step go where marked ``TODO``). Until then the sim2real *leg* is
structurally unified but functionally absent.
"""

from __future__ import annotations

from typing import Optional

import numpy as np

from ..types import (
    ControlMode,
    GripperCommand,
    RobotState,
)
from .base import RobotBackend


class MujocoBackend(RobotBackend):
    # NOTE: model_path defaults to None so the backend is constructible through
    # the Robot()/get_backend factory and CLI without a TypeError. It is a stub:
    # connect() fails fast (see below) until the model load + IK/OSC mapping are
    # implemented, rather than reporting a misleading successful connection.
    def __init__(self, model_path: Optional[str] = None, n_joints: int = 7, control_dt: float = 0.001):
        self.model_path = model_path
        self.n_joints = n_joints
        self.dt = control_dt
        self._connected = False
        self._mode = ControlMode.IDLE
        self._model = None
        self._data = None

    def connect(self) -> None:
        try:
            import mujoco  # noqa: F401
        except Exception as exc:  # pragma: no cover
            raise ImportError("MujocoBackend needs `pip install mujoco`") from exc
        # Fail loudly and early: this is a deliberate stub. read_state()/stream_*
        # are not implemented, so a "successful" connect() would be misleading.
        # TODO: self._model = mujoco.MjModel.from_xml_path(self.model_path)
        #       self._data = mujoco.MjData(self._model); self._connected = True
        raise NotImplementedError(
            "MujocoBackend is a stub: provide your Rizon MJCF as `model_path` and "
            "implement the model load + IK/OSC mapping (read_state/stream_*) in "
            "backends/mujoco.py before using the 'mujoco' backend."
        )

    def disconnect(self) -> None:
        self._connected = False

    @property
    def is_connected(self) -> bool:
        return self._connected

    def read_state(self) -> RobotState:
        # TODO: read qpos/qvel/site pose from self._data and fill RobotState.
        return RobotState(control_mode=self._mode)

    def set_mode(self, mode: ControlMode, **kwargs) -> None:  # noqa: D401
        self._mode = mode

    def stream_cartesian(self, pose: np.ndarray, wrench: Optional[np.ndarray] = None) -> None:
        # TODO: IK / operational-space step toward `pose`, then mj_step.
        raise NotImplementedError("fill in MuJoCo IK / OSC mapping")

    def stream_joint(self, q: np.ndarray) -> None:
        # TODO: set self._data.ctrl (position actuators) and mj_step.
        raise NotImplementedError("fill in MuJoCo joint actuation")

    def move_gripper(self, cmd: GripperCommand) -> None:
        pass

    def stop(self) -> None:
        self._mode = ControlMode.IDLE
