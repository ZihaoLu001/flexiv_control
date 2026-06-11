"""Tests for URDF asset generation (mimic flattening, mesh-path rewriting).

The postprocess logic is pure stdlib and always runs; the loading/articulation
tests are gated on yourdfpy and on a generated URDF being present in the user
cache (produced by ``flexiv-control viz --fetch-assets`` or the asset API).
"""

from __future__ import annotations

import numpy as np
import pytest

from flexiv_control.viz import assets

_SYNTH = """<?xml version="1.0"?>
<robot name="t">
  <link name="base"/>
  <link name="a"/><link name="b"/><link name="c"/>
  <joint name="drive" type="revolute">
    <parent link="base"/><child link="a"/>
    <axis xyz="0 0 1"/><limit lower="0" upper="1" effort="1" velocity="1"/>
  </joint>
  <joint name="first" type="revolute">
    <parent link="a"/><child link="b"/>
    <axis xyz="0 0 1"/><limit lower="-10" upper="10" effort="1" velocity="1"/>
    <mimic joint="drive" multiplier="2.0" offset="0.1"/>
  </joint>
  <joint name="second" type="revolute">
    <parent link="b"/><child link="c"/>
    <axis xyz="0 0 1"/><limit lower="-10" upper="10" effort="1" velocity="1"/>
    <mimic joint="first" multiplier="-1.0" offset="0.05"/>
  </joint>
  <link name="m">
    <visual><geometry>
      <mesh filename="package://some_pkg/meshes/x.obj"/>
    </geometry></visual>
    <collision><geometry>
      <mesh filename="meshes/y.stl"/>
    </geometry></collision>
  </link>
</robot>
"""


def test_postprocess_flattens_nested_mimics(tmp_path):
    out = assets._postprocess_urdf(_SYNTH, tmp_path)
    import xml.etree.ElementTree as ET

    root = ET.fromstring(out)
    mimics = {j.get("name"): j.find("mimic") for j in root.iter("joint")
              if j.find("mimic") is not None}
    # one-level mimic untouched
    assert mimics["first"].get("joint") == "drive"
    # nested mimic now points at the ACTUATED drive with composed coefficients:
    # second = -1*first + 0.05 = -1*(2*drive + 0.1) + 0.05 = -2*drive - 0.05
    assert mimics["second"].get("joint") == "drive"
    assert float(mimics["second"].get("multiplier")) == pytest.approx(-2.0)
    assert float(mimics["second"].get("offset")) == pytest.approx(-0.05)


def test_postprocess_absolutizes_mesh_paths(tmp_path):
    out = assets._postprocess_urdf(_SYNTH, tmp_path)
    root = tmp_path.as_posix()
    assert f"{root}/meshes/x.obj" in out      # package:// -> checkout-absolute
    assert f"{root}/meshes/y.stl" in out      # relative   -> checkout-absolute
    assert "package://" not in out


@pytest.mark.skipif(
    assets.ensure_rizon_urdf() is None,
    reason="no generated Rizon URDF in the cache (run flexiv-control viz --fetch-assets)",
)
def test_generated_urdf_articulates_gripper():
    yourdfpy = pytest.importorskip("yourdfpy")  # noqa: F841
    urdf = assets.load_urdf(assets.ensure_rizon_urdf())
    names = list(urdf.actuated_joint_names)
    # 7 arm joints + the gripper width drive
    assert {f"joint{i}" for i in range(1, 8)} <= set(names)
    assert "finger_width_joint" in names
    q = np.zeros(len(names))
    wj = names.index("finger_width_joint")

    def tip_gap(width: float) -> float:
        q[wj] = width
        urdf.update_cfg(q)
        L = urdf.get_transform("left_finger_tip", "grav_base_link")[:3, 3]
        R = urdf.get_transform("right_finger_tip", "grav_base_link")[:3, 3]
        return float(np.linalg.norm(L - R))

    spread = tip_gap(0.085) - tip_gap(0.0)
    # BOTH fingers must articulate (the un-flattened mimic chain froze five of
    # six finger joints and produced ~half this spread on one side only).
    assert spread > 0.06
