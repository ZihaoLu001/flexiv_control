"""Thin network client(s) for the control server."""

from .remote_robot import RemoteRobot, RemoteRobotError  # noqa: F401

__all__ = ["RemoteRobot", "RemoteRobotError"]
