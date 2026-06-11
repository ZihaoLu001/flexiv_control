"""Tiny, dependency-free SE(3) / quaternion helpers.

We intentionally avoid pulling in scipy/transforms3d into the core so the
client stays light enough to ``pip install`` into any project. Quaternions are
``(w, x, y, z)`` everywhere (Flexiv convention).
"""

from __future__ import annotations

import numpy as np

_EPS = 1e-9


def quat_normalize(q: np.ndarray) -> np.ndarray:
    q = np.asarray(q, float).reshape(4)
    n = np.linalg.norm(q)
    if n < _EPS:
        return np.array([1.0, 0.0, 0.0, 0.0])
    q = q / n
    # canonicalise sign (w >= 0) to avoid double-cover ambiguity
    if q[0] < 0:
        q = -q
    return q


def quat_mul(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array(
        [
            aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
        ]
    )


def quat_conj(q: np.ndarray) -> np.ndarray:
    return np.array([q[0], -q[1], -q[2], -q[3]])


def quat_angle(a: np.ndarray, b: np.ndarray) -> float:
    """Geodesic angle (radians) between two orientations."""
    a = quat_normalize(a)
    b = quat_normalize(b)
    d = abs(float(np.dot(a, b)))
    d = min(1.0, max(-1.0, d))
    return 2.0 * np.arccos(d)


def quat_slerp(a: np.ndarray, b: np.ndarray, t: float) -> np.ndarray:
    a = quat_normalize(a)
    b = quat_normalize(b)
    dot = float(np.dot(a, b))
    if dot < 0.0:
        b = -b
        dot = -dot
    if dot > 0.9995:  # nearly parallel -> linear interp
        return quat_normalize(a + t * (b - a))
    theta_0 = np.arccos(min(1.0, max(-1.0, dot)))
    sin_0 = np.sin(theta_0)
    theta = theta_0 * t
    s0 = np.sin(theta_0 - theta) / sin_0
    s1 = np.sin(theta) / sin_0
    return quat_normalize(s0 * a + s1 * b)


def quat_to_rotvec(q: np.ndarray) -> np.ndarray:
    """Quaternion -> rotation vector (axis * angle)."""
    q = quat_normalize(q)
    w = min(1.0, max(-1.0, q[0]))
    angle = 2.0 * np.arccos(w)
    s = np.sqrt(max(0.0, 1.0 - w * w))
    if s < _EPS:
        return np.zeros(3)
    return (q[1:] / s) * angle


def rotvec_to_quat(rv: np.ndarray) -> np.ndarray:
    rv = np.asarray(rv, float).reshape(3)
    angle = float(np.linalg.norm(rv))
    if angle < _EPS:
        return np.array([1.0, 0.0, 0.0, 0.0])
    axis = rv / angle
    h = angle / 2.0
    return np.concatenate([[np.cos(h)], np.sin(h) * axis])


def quat_rotate(q: np.ndarray, v: np.ndarray) -> np.ndarray:
    """Rotate a 3-vector ``v`` by quaternion ``q`` (active rotation, (w,x,y,z))."""
    q = quat_normalize(q)
    v = np.asarray(v, float).reshape(3)
    qv = np.concatenate([[0.0], v])
    return quat_mul(quat_mul(q, qv), quat_conj(q))[1:]


def top_down_quat(yaw: float, *, base_quat=(0.0, 0.0, 1.0, 0.0)) -> np.ndarray:
    """Tool-pointing-down orientation, yawed about the base z axis.

    The canonical tabletop-manipulation orientation: ``q = rotz(yaw) * base_quat``
    where ``base_quat`` (w, x, y, z) is YOUR tool-down quaternion (the default,
    a 180-degree flip about base y, suits a flange whose tool axis is +z). Use
    this instead of hand-composing the quaternion in every top-down planner.
    """
    half = 0.5 * float(yaw)
    qy = np.array([np.cos(half), 0.0, 0.0, np.sin(half)])
    return quat_normalize(quat_mul(qy, np.asarray(base_quat, float).reshape(4)))


def mat_to_quat(R: np.ndarray) -> np.ndarray:
    """3x3 rotation matrix -> quaternion (w, x, y, z), Shepperd's method."""
    R = np.asarray(R, float).reshape(3, 3)
    t = float(np.trace(R))
    if t > 0.0:
        s = np.sqrt(t + 1.0) * 2.0
        q = np.array(
            [0.25 * s, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s]
        )
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        q = np.array(
            [(R[2, 1] - R[1, 2]) / s, 0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s]
        )
    elif R[1, 1] > R[2, 2]:
        s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        q = np.array(
            [(R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s]
        )
    else:
        s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        q = np.array(
            [(R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s]
        )
    return quat_normalize(q)


def integrate_pose(pose: np.ndarray, delta: np.ndarray, frame: str = "base") -> np.ndarray:
    """Integrate a 6-vector delta ``[dx,dy,dz,rx,ry,rz]`` onto a 7-pose.

    The rotation part ``[rx,ry,rz]`` is an axis-angle *rotation vector* and always
    right-multiplies in the body (TCP) frame -- the standard OSC / delta-pose
    convention used by robosuite-style environments. The translation part is
    interpreted in ``frame``:

    * ``"base"`` (default): the delta adds directly in the robot base frame.
    * ``"tcp"``: the delta is first rotated into the base frame by the current
      TCP orientation, so a ``+x`` delta means "forward along the gripper".
    """
    pose = np.asarray(pose, float).reshape(7)
    delta = np.asarray(delta, float).reshape(6)
    out = pose.copy()
    d_trans = delta[:3]
    if frame == "tcp":
        d_trans = quat_rotate(pose[3:7], d_trans)
    elif frame != "base":
        raise ValueError(f"integrate_pose: unsupported frame {frame!r} (expected 'base' or 'tcp')")
    out[:3] = pose[:3] + d_trans
    dq = rotvec_to_quat(delta[3:])
    out[3:7] = quat_normalize(quat_mul(pose[3:7], dq))
    return out
