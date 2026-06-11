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
    """Resolve a Rizon URDF path, or ``None`` (-> frames mode).

    ``download=True`` permits fetching the pinned ``flexiv_description``
    tarball into the user cache when nothing is found locally.
    """
    env = os.environ.get("FLEXIV_DESCRIPTION_DIR")
    if env:
        found = _find_urdf(Path(env), model)
        if found is not None:
            return found
    found = _find_urdf(cache_dir(), model)
    if found is not None:
        return found
    if download:
        root = fetch_flexiv_description()
        if root is not None:
            return _find_urdf(root, model)
    return None


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
