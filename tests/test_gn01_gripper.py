"""Build the Rizon + GN01 combined model and drive the gripper.

Runs when mujoco + a Rizon arm MJCF (FLEXIV_RIZON_MJCF, e.g. menagerie
flexiv_rizon4s/scene.xml) + the Grav meshes (FLEXIV_GRAV_MESHDIR, e.g.
flexiv_description/meshes/Grav) are available. Skips in CI."""

from __future__ import annotations

import os

import numpy as np
import pytest

_ARM = os.environ.get("FLEXIV_RIZON_MJCF")
_GRAV = os.environ.get("FLEXIV_GRAV_MESHDIR")
pytestmark = pytest.mark.skipif(
    not (_ARM and _GRAV),
    reason="set FLEXIV_RIZON_MJCF + FLEXIV_GRAV_MESHDIR to run the GN01 gripper test",
)


def _build(tmp_path):
    pytest.importorskip("mujoco")
    from flexiv_control.sim import build

    return build(_ARM, _GRAV, tmp_path / "rizon_gn01.xml")


def test_build_has_gripper_actuator_and_tcp(tmp_path):
    import mujoco

    out = _build(tmp_path)
    assert out.exists()
    m = mujoco.MjModel.from_xml_path(str(out))
    assert mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, "gn01_width") >= 0
    assert mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, "tcp") >= 0


def test_gn01_physically_opens_and_closes(tmp_path):
    import mujoco

    out = _build(tmp_path)
    m = mujoco.MjModel.from_xml_path(str(out))
    d = mujoco.MjData(m)
    a = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, "gn01_width")
    lid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, "left_finger_tcp")
    rid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, "right_finger_tcp")
    assert a >= 0 and lid >= 0 and rid >= 0

    def gap_at(w):
        d.ctrl[a] = 9.404 * w - 0.155
        for _ in range(800):
            mujoco.mj_step(m, d)
        return float(np.linalg.norm(d.site_xpos[lid] - d.site_xpos[rid]))

    assert gap_at(0.0) < 0.01          # closed
    assert abs(gap_at(0.10) - 0.10) < 0.01  # fully open ~ 0.10 m


def test_backend_drives_gripper_width(tmp_path):
    out = _build(tmp_path)
    from flexiv_control import GripperCommand, Robot, RobotConfig

    r = Robot(
        RobotConfig(
            backend="mujoco", model_path=str(out), control_hz=100.0,
            mujoco_tcp_site="tcp", mujoco_gripper_actuator="gn01_width",
            mujoco_gripper_width_scale=9.404, mujoco_gripper_width_offset=-0.155,
        )
    )
    r.connect()
    r.home()
    r.start_cartesian_impedance()
    s = r.get_state()
    r.command_gripper(GripperCommand(width=0.08))
    for _ in range(40):  # step the sim by holding the current pose
        r.backend.stream_cartesian(s.tcp_pose)
    assert abs(r.get_state().gripper_width - 0.08) < 0.02  # width round-trips
    r.disconnect()
