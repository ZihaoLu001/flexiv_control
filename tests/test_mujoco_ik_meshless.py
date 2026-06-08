"""Mesh-free regression test for the MuJoCo backend's DLS Cartesian IK.

The full sim test (test_mujoco_backend.py) needs the Rizon MJCF + meshes and is
skipped in CI. This one builds a tiny inline planar 3R arm from an XML string --
no meshes, no env var -- so the IK math (Jacobian + damped least squares) is
regression-guarded in CI (the `sim` job installs mujoco). It asserts the TCP
converges to a reachable Cartesian target."""

from __future__ import annotations

import numpy as np
import pytest

pytest.importorskip("mujoco")

from flexiv_control.backends.mujoco import MujocoBackend  # noqa: E402

# Planar 3R arm in the XY plane (gravity off): three hinge joints about z, three
# 0.25 m links, a `tcp` site at the end, position actuators. Fully mesh-free.
_ARM_XML = """
<mujoco>
  <option timestep="0.002" gravity="0 0 0"/>
  <default>
    <joint type="hinge" axis="0 0 1" damping="2.0" armature="0.1"/>
    <geom type="capsule" size="0.02"/>
    <position kp="40" kv="6" ctrlrange="-3.14 3.14"/>
  </default>
  <worldbody>
    <body name="l1" pos="0 0 0">
      <joint name="j1"/>
      <geom fromto="0 0 0 0.25 0 0"/>
      <body name="l2" pos="0.25 0 0">
        <joint name="j2"/>
        <geom fromto="0 0 0 0.25 0 0"/>
        <body name="l3" pos="0.25 0 0">
          <joint name="j3"/>
          <geom fromto="0 0 0 0.25 0 0"/>
          <site name="tcp" pos="0.25 0 0" size="0.01"/>
        </body>
      </body>
    </body>
  </worldbody>
  <actuator>
    <position joint="j1"/>
    <position joint="j2"/>
    <position joint="j3"/>
  </actuator>
</mujoco>
"""


def _backend(tmp_path):
    p = tmp_path / "arm3r.xml"
    p.write_text(_ARM_XML)
    b = MujocoBackend(
        model_path=str(p), n_joints=3, control_dt=0.02,
        tcp_site="tcp", gravity_comp=False, ik_gain=0.3,
    )
    b.connect()
    return b


def test_dls_ik_converges_to_reachable_target(tmp_path):
    b = _backend(tmp_path)
    # Nudge off the singular fully-extended start so the Jacobian is well-conditioned.
    b._d.qpos[b._qadr] = [0.3, 0.4, 0.3]
    b._mj.mj_forward(b._m, b._d)
    b._d.ctrl[b._act] = [0.3, 0.4, 0.3]

    start_pos = b.read_state().tcp_pose[:3].copy()
    # A reachable in-plane target (arm reach is 0.75 m); keep orientation = current
    # so the 3R (planar) arm isn't asked for an impossible out-of-plane rotation.
    quat = b.read_state().tcp_pose[3:7]
    target = np.array([0.35, 0.30, 0.0])
    pose = np.concatenate([target, quat])

    err0 = float(np.linalg.norm(b.read_state().tcp_pose[:3] - target))
    for _ in range(400):
        b.stream_cartesian(pose)
    errN = float(np.linalg.norm(b.read_state().tcp_pose[:3] - target))

    assert errN < 0.01                       # converged to within 1 cm
    assert errN < 0.2 * err0                 # and dramatically reduced the error
    assert np.linalg.norm(start_pos - target) > 0.05  # the target was non-trivially far
    b.disconnect()
