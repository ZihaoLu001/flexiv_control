"""MuJoCo backend (stub) for sim and real2sim2real.

Why this matters for RL / planning: the *same* ``CartesianChunk`` / ``CartesianDelta``
action and the *same* ``RobotState`` observation are used in sim and on the real
robot. That is the contract that makes real2sim2real and sim-trained policies
transfer with no rewrite -- exactly the property Deoxys (robosuite->real) and
Polymetis (pybullet->real) are valued for.

This is a deliberate stub: it defines the structure and the integration points
but leaves the model load + IK / operational-space mapping for you to fill in
against your Rizon MJCF. The Flexiv
``flexiv_description`` URDF / your MuJoCo model and a Jacobian-based or
``mujoco`` MJX IK step go where marked ``TODO``.
"""

from __future__ import annotations

from typing import Optional

import numpy as np

from ..types import (
    CART_DOF,
    ControlMode,
    ForceControlParams,
    GripperCommand,
    ImpedanceParams,
    JointImpedanceParams,
    RobotState,
)
from .base import RobotBackend


class MujocoBackend(RobotBackend):
    def __init__(self, model_path: str, n_joints: int = 7, control_dt: float = 0.001):
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
        # TODO: self._model = mujoco.MjModel.from_xml_path(self.model_path)
        #       self._data = mujoco.MjData(self._model)
        self._connected = True

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
