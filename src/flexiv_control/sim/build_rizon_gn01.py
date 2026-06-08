"""Build a combined Rizon + GN01 (Grav) MuJoCo model.

Attaches the GN01/Grav parallel-jaw gripper (``grav_gn01.xml``, faithful 4-bar
with the real Flexiv meshes) onto a Rizon arm MJCF at the flange, producing a
persistent arm+gripper model whose geometry matches the real lab setup. Needs:

* a Rizon arm MJCF -- the official DeepMind menagerie ``flexiv_rizon4/scene.xml``
  (or ``flexiv_rizon4s``);
* the Flexiv Grav meshes -- ``flexiv_description/meshes/Grav`` (Apache-2.0).

CLI::

    python -m flexiv_control.sim.build_rizon_gn01 \
        --arm  <menagerie>/flexiv_rizon4/scene.xml \
        --grav-meshdir <flexiv_description>/meshes/Grav \
        --out  rizon4_gn01.xml

Then point the mujoco backend at it (the lab config already references it)::

    RobotConfig(backend="mujoco", model_path="rizon4_gn01.xml",
                mujoco_tcp_site="tcp", mujoco_gripper_actuator="gn01_width",
                mujoco_gripper_width_scale=9.404, mujoco_gripper_width_offset=-0.155)
"""

from __future__ import annotations

import argparse
import math
import os
from pathlib import Path

_TEMPLATE = Path(__file__).with_name("grav_gn01.xml")

# The official Flexiv flange transform (link7 -> flange / grav_base_link):
# xyz [0, 0, 0.124], rpy [0, 0, -pi]  (flexiv_description Rizon4s default_kinematics).
DEFAULT_FLANGE_XYZ = (0.0, 0.0, 0.124)
DEFAULT_FLANGE_RPY = (0.0, 0.0, -math.pi)


def _quat_from_rpy(r: float, p: float, y: float):
    cr, sr = math.cos(r / 2), math.sin(r / 2)
    cp, sp = math.cos(p / 2), math.sin(p / 2)
    cy, sy = math.cos(y / 2), math.sin(y / 2)
    return [
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    ]


def _absolutize_meshes(spec, base_dir: str) -> None:
    """Rewrite a spec's mesh file paths to absolute and clear its meshdir, so the
    serialized model loads regardless of where the output file is written."""
    md = spec.meshdir or ""
    root = md if os.path.isabs(md) else os.path.join(base_dir, md)
    for mesh in spec.meshes:
        if mesh.file and not os.path.isabs(mesh.file):
            mesh.file = os.path.abspath(os.path.join(root, mesh.file))
    spec.meshdir = ""


def build(
    arm_xml,
    grav_meshdir,
    out_path,
    *,
    parent_body: str = "link7",
    flange_xyz=DEFAULT_FLANGE_XYZ,
    flange_rpy=DEFAULT_FLANGE_RPY,
    prefix: str = "",
) -> Path:
    """Attach the GN01 gripper to ``arm_xml`` at ``parent_body``+flange; write the
    combined MJCF to ``out_path`` and return it."""
    import mujoco

    tmpl = _TEMPLATE.read_text(encoding="utf-8").replace(
        "__GRAV_MESHDIR__", str(Path(grav_meshdir))
    )
    spec_grip = mujoco.MjSpec.from_string(tmpl)
    spec_arm = mujoco.MjSpec.from_file(str(arm_xml))
    # Make every mesh path absolute before attaching, so the combined model is
    # relocatable (arm meshes are relative to the menagerie dir; gripper meshes
    # to the Grav meshdir).
    _absolutize_meshes(spec_grip, "")
    _absolutize_meshes(spec_arm, os.path.dirname(os.path.abspath(str(arm_xml))))
    parent = spec_arm.body(parent_body)
    if parent is None:
        raise ValueError(f"parent body {parent_body!r} not found in {arm_xml}")
    frame = parent.add_frame()
    frame.pos = list(flange_xyz)
    frame.quat = _quat_from_rpy(*flange_rpy)
    frame.attach_body(spec_grip.body("grav_base_link"), prefix, "")
    spec_arm.compile()  # validate it builds
    xml = spec_arm.to_xml()
    # mjSpec can emit a spurious empty nested <default/> (from the attached
    # gripper's implicit main default class). A class-less nested <default> is
    # invalid and fails to reload, so drop those lines.
    xml = "\n".join(
        ln for ln in xml.splitlines()
        if ln.strip() not in ("<default/>", "<default></default>")
    )
    out = Path(out_path)
    out.write_text(xml, encoding="utf-8")
    return out


def main(argv=None) -> None:
    ap = argparse.ArgumentParser(description="Build a Rizon + GN01 combined MJCF.")
    ap.add_argument("--arm", required=True, help="Rizon arm MJCF (menagerie flexiv_rizon4/scene.xml)")
    ap.add_argument("--grav-meshdir", required=True, help="flexiv_description/meshes/Grav")
    ap.add_argument("--out", required=True, help="output combined MJCF path")
    ap.add_argument("--parent-body", default="link7", help="arm body to mount on (default link7)")
    a = ap.parse_args(argv)
    print("wrote", build(a.arm, a.grav_meshdir, a.out, parent_body=a.parent_body))


if __name__ == "__main__":
    main()
