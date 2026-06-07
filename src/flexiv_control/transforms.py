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


def integrate_pose(pose: np.ndarray, delta: np.ndarray) -> np.ndarray:
    """Integrate a 6-vector delta ``[dx,dy,dz,drx,dry,drz]`` onto a 7-pose.

    Translation adds in the given (base) frame; rotation right-multiplies in the
    body frame, which matches the standard OSC / delta-pose convention used by
    robosuite-style environments.
    """
    pose = np.asarray(pose, float).reshape(7)
    delta = np.asarray(delta, float).reshape(6)
    out = pose.copy()
    out[:3] = pose[:3] + delta[:3]
    dq = rotvec_to_quat(delta[3:])
    out[3:7] = quat_normalize(quat_mul(pose[3:7], dq))
    return out
