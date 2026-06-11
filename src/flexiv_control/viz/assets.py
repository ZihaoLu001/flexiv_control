"""Locate (or fetch) the Rizon URDF + meshes for the cosmetic robot mirror.

The mirror is COSMETIC: link visuals only. The TCP marker, trail, and planned
path are always drawn from the robot-streamed ``tcp_pose``, never from local
URDF forward kinematics -- Flexiv's published URDFs have accurate kinematics
for rendering but flexiv_rdk issue #82 documented a >4 cm URDF-vs-robot TCP
discrepancy, so FK is not authoritative for the tool point.

Resolution order (first hit wins):
1. ``FLEXIV_DESCRIPTION_DIR`` -- a local checkout of
   https://github.com/flexivrobotics/flexiv_description (branch ``humble-v1``
   or ``jazzy-v1``; the ``*-v2`` branches are a DIFFERENT robot, not Rizon).
2. The user cache (``~/.cache/flexiv_control/``), populated by a previous
   :func:`fetch_flexiv_description` call.
3. ``fetch_flexiv_description()`` downloads the pinned branch tarball into the
   cache -- only when explicitly requested (the CLI asks; the library never
   downloads silently).

If no URDF can be resolved the viewer runs in "frames mode" (TCP frame, trail,
workspace box, planned path, HUD) -- fully functional for the safety-preview
use case; assets are never load-bearing.
"""

from __future__ import annotations

import os
import tarfile
import tempfile
import urllib.request
from pathlib import Path
from typing import Optional

# Pin the Rizon-era branch: humble-v2/jazzy-v2 describe the newer EnlightL
# robot, not the Rizon 4s.
FLEXIV_DESCRIPTION_REF = os.environ.get("FLEXIV_DESCRIPTION_REF", "humble-v1")
FLEXIV_DESCRIPTION_TARBALL = (
    "https://github.com/flexivrobotics/flexiv_description/archive/refs/heads/"
    f"{FLEXIV_DESCRIPTION_REF}.tar.gz"
)


def cache_dir() -> Path:
    return Path(os.path.expanduser("~")) / ".cache" / "flexiv_control"


def _find_urdf(root: Path, model: str) -> Optional[Path]:
    """Find a URDF for ``model`` under ``root``. Prefers exact-model matches
    (e.g. ``rizon4s``) and 'kinematics' variants; tolerates both committed
    URDFs and ones a user generated from the xacro sources."""
    if not root.is_dir():
        return None
    model = model.lower()
    candidates = sorted(p for p in root.rglob("*.urdf") if model in p.name.lower())
    if not candidates:
        # e.g. model="rizon4s" but only "rizon4" files exist -- better than nothing,
        # the link visuals are near-identical.
        base = model.rstrip("s")
        candidates = sorted(p for p in root.rglob("*.urdf") if base in p.name.lower())
    if not candidates:
        return None
    # Prefer "kinematics" URDFs (the RDK-published flavor), then shortest name.
    candidates.sort(key=lambda p: (0 if "kinematics" in p.name.lower() else 1, len(p.name)))
    return candidates[0]


def ensure_rizon_urdf(model: str = "rizon4s", *, download: bool = False) -> Optional[Path]:
    """Resolve a Rizon URDF path (arm + articulated GN01 gripper), or ``None``
    (-> frames mode).

    flexiv_description ships xacro SOURCES, not a committed URDF, so the
    resolution chain is: (1) a previously generated URDF in the cache; (2) any
    ``*.urdf`` the user provides under ``FLEXIV_DESCRIPTION_DIR`` or the
    cache; (3) generate one from the xacro sources (standalone ``xacro``
    package, no ROS) found in the env-var checkout or the cache;
    (4) with ``download=True``, fetch the pinned checkout first.
    """
    gen = cache_dir() / "generated" / f"{model}_gn01.urdf"
    if gen.is_file():
        return gen
    env = os.environ.get("FLEXIV_DESCRIPTION_DIR")
    if env:
        found = _find_urdf(Path(env), model)
        if found is not None:
            return found
    found = _find_urdf(cache_dir(), model)
    if found is not None:
        return found
    for root in filter(None, [Path(env) if env else None, _cached_checkout()]):
        out = generate_rizon_urdf(root, model=model)
        if out is not None:
            return out
    if download:
        root = fetch_flexiv_description()
        if root is not None:
            return _find_urdf(root, model) or generate_rizon_urdf(root, model=model)
    return None


def _cached_checkout() -> Optional[Path]:
    roots = sorted(cache_dir().glob("flexiv_description-*"))
    return roots[-1] if roots else None


def generate_rizon_urdf(
    checkout: Path, *, model: str = "rizon4s", load_gripper: bool = True
) -> Optional[Path]:
    """Generate ``<model>`` URDF (with the articulated Flexiv-GN01 gripper)
    from a flexiv_description checkout, using the standalone ``xacro`` package.

    The xacro sources reference the package via ROS ``$(find
    flexiv_description)`` substitutions, which standalone xacro cannot
    resolve; we shadow-copy the tree and substitute the literal path first.
    The result is cached at ``~/.cache/flexiv_control/generated/`` and ends up
    with ABSOLUTE mesh paths into the shadow copy, so it loads in yourdfpy
    with no package handling. Returns ``None`` (frames mode) on any failure.
    """
    try:
        import shutil

        import xacro  # standalone PyPI package; no ROS needed

        checkout = Path(checkout)
        entry = checkout / "urdf" / "rizon.urdf.xacro"
        if not entry.is_file():
            return None
        shadow = cache_dir() / "generated" / "_xacro_shadow" / checkout.name
        if shadow.exists():
            shutil.rmtree(shadow)
        shadow.parent.mkdir(parents=True, exist_ok=True)
        shutil.copytree(checkout, shadow)
        token = "$(find flexiv_description)"
        for f in shadow.rglob("*.xacro"):
            text = f.read_text(encoding="utf-8")
            if token in text:
                f.write_text(text.replace(token, shadow.as_posix()), encoding="utf-8")
        # The vendor xacro spells types capitalized (Rizon4s).
        rizon_type = model[0].upper() + model[1:].lower()
        doc = xacro.process_file(
            str(shadow / "urdf" / "rizon.urdf.xacro"),
            mappings={
                "rizon_type": rizon_type,
                "load_gripper": "true" if load_gripper else "false",
            },
        )
        urdf_xml = _postprocess_urdf(doc.toprettyxml(indent="  "), shadow)
        out = cache_dir() / "generated" / f"{model}_gn01.urdf"
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(urdf_xml, encoding="utf-8")
        return out
    except Exception as e:  # noqa: BLE001 -- assets are never load-bearing
        print(f"[flexiv_control.viz] URDF generation failed "
              f"({type(e).__name__}: {e}); running in frames mode")
        return None


def _postprocess_urdf(urdf_xml: str, package_root: Path) -> str:
    """Make the generated URDF self-contained for yourdfpy:

    1. **Absolutize mesh paths.** Some vendor visual parameters emit
       package-relative mesh filenames (``meshes/Rizon4s/visual/link7.obj``);
       a loader would resolve those against the URDF's own directory (the
       cache), not the checkout. Prefix every relative, non-``package://``
       filename with the shadow checkout root.
    2. **Flatten nested mimics.** The GN01 gripper chains its mimics (e.g.
       ``left_inner_knuckle`` mimics ``left_outer_knuckle`` which mimics the
       actuated ``finger_width_joint``), but yourdfpy resolves only ONE level
       -- nested mimics collapse to ``0.0 + offset`` (with a warning per
       update), freezing five of the six finger joints. Composing
       transitively (if A = m1*B + o1 and B = m2*C + o2 then
       A = (m1*m2)*C + (m1*o2 + o1)) points every mimic at the actuated drive
       joint, so the whole 4-bar articulates.
    """
    import xml.etree.ElementTree as ET

    root = ET.fromstring(urdf_xml)

    for mesh in root.iter("mesh"):
        fname = mesh.get("filename", "")
        if not fname or os.path.isabs(fname):
            continue
        if fname.startswith("package://"):
            rel = fname[len("package://"):]
            rel = rel.split("/", 1)[1] if "/" in rel else rel  # drop the package name
            mesh.set("filename", (package_root / rel).as_posix())
        else:
            mesh.set("filename", (package_root / fname).as_posix())
    mimics = {}      # joint name -> (target, multiplier, offset, element)
    actuated = set()
    for joint in root.iter("joint"):
        name = joint.get("name", "")
        m = joint.find("mimic")
        if m is not None:
            mimics[name] = (
                m.get("joint", ""),
                float(m.get("multiplier", "1") or 1.0),
                float(m.get("offset", "0") or 0.0),
                m,
            )
        elif joint.get("type") not in ("fixed", None):
            actuated.add(name)
    for name, (target, mult, off, elem) in mimics.items():
        depth = 0
        while target in mimics and depth < 8:
            t2, m2, o2, _ = mimics[target]
            mult, off, target = mult * m2, mult * o2 + off, t2
            depth += 1
        if target in actuated:
            elem.set("joint", target)
            elem.set("multiplier", repr(mult))
            elem.set("offset", repr(off))
    return ET.tostring(root, encoding="unicode")


def fetch_flexiv_description(timeout: float = 60.0) -> Optional[Path]:
    """Download the pinned flexiv_description branch into the cache and return
    the extracted root (Apache-2.0, vendor-published). Returns ``None`` on any
    network/extraction failure -- asset fetching must never crash the viewer."""
    dest = cache_dir()
    try:
        dest.mkdir(parents=True, exist_ok=True)
        with tempfile.NamedTemporaryFile(suffix=".tar.gz", delete=False) as tmp:
            with urllib.request.urlopen(FLEXIV_DESCRIPTION_TARBALL, timeout=timeout) as r:
                tmp.write(r.read())
            tmp_path = Path(tmp.name)
        with tarfile.open(tmp_path, "r:gz") as tf:
            # Guard against path traversal in the archive.
            for m in tf.getmembers():
                target = (dest / m.name).resolve()
                if not str(target).startswith(str(dest.resolve())):
                    raise RuntimeError(f"unsafe path in archive: {m.name}")
            tf.extractall(dest)
        tmp_path.unlink(missing_ok=True)
        roots = sorted(dest.glob("flexiv_description-*"))
        return roots[-1] if roots else dest
    except Exception as e:  # noqa: BLE001  -- degrade to frames mode, loudly
        print(f"[flexiv_control.viz] asset fetch failed ({type(e).__name__}: {e}); "
              "running in frames mode. Set FLEXIV_DESCRIPTION_DIR to a local "
              "checkout of flexivrobotics/flexiv_description to enable the mesh mirror.")
        return None


def load_urdf(urdf_path: Path):
    """Load a URDF with yourdfpy, resolving both relative and ROS
    ``package://flexiv_description/...`` mesh URIs against the checkout."""
    import yourdfpy

    urdf_path = Path(urdf_path)
    # The package root is the directory that CONTAINS meshes/ -- walk up from
    # the URDF until we find it (URDFs live at varying depths across branches).
    pkg_root = urdf_path.parent
    for cand in [urdf_path.parent, *urdf_path.parents]:
        if (cand / "meshes").is_dir():
            pkg_root = cand
            break

    def _handler(fname: str) -> str:
        if fname.startswith("package://"):
            rel = fname[len("package://"):]
            # strip the leading package name (e.g. "flexiv_description/")
            rel = rel.split("/", 1)[1] if "/" in rel else rel
            return str(pkg_root / rel)
        if not os.path.isabs(fname):
            return str((urdf_path.parent / fname).resolve())
        return fname

    return yourdfpy.URDF.load(
        str(urdf_path),
        filename_handler=_handler,
        load_collision_meshes=False,
    )
