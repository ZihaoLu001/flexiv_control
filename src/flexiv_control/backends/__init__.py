from .base import RobotBackend
from .fake import FakeBackend

__all__ = ["RobotBackend", "FakeBackend"]

# FlexivRdkBackend and MujocoBackend are imported lazily by name to avoid
# importing optional heavy/optional deps at package import time.


def get_backend(name: str, **kwargs) -> RobotBackend:
    """Factory: ``get_backend("fake")``, ``get_backend("flexiv_rdk", robot_sn=...)``."""
    name = name.lower()
    if name in ("fake", "sim_kinematic"):
        return FakeBackend(**kwargs)
    if name in ("flexiv_rdk", "rdk", "flexiv"):
        from .flexiv_rdk import FlexivRdkBackend

        return FlexivRdkBackend(**kwargs)
    if name in ("mujoco", "mjx"):
        from .mujoco import MujocoBackend

        return MujocoBackend(**kwargs)
    raise ValueError(f"unknown backend: {name!r}")
