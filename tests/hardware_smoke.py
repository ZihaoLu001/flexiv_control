#!/usr/bin/env python3
"""Read-only hardware smoke test for a real Flexiv Rizon.

This is the FIRST thing to run on a new robot/RDK install, before you command
any motion. It is deliberately **read-only by default**: it connects, enables,
waits until operational, and prints the live state a few times. It never moves
the arm unless you pass ``--allow-motion`` (and even then only a tiny, bounded
hold at the current pose).

It also surfaces, in one place, every RDK call this library depends on, so a
version mismatch shows up here as a clear error rather than mid-trajectory.

Usage
-----
    # read-only (safe): connect, enable, print state
    python tests/hardware_smoke.py --sn Rizon4s-XXXXXX

    # also exercise a 2 s hold at the CURRENT pose (clear the workspace first!)
    python tests/hardware_smoke.py --sn Rizon4s-XXXXXX --allow-motion

Prerequisites
-------------
* ``pip install flexivrdk`` matching your robot software version (this library
  targets RDK **v1.x**; the v2.x dict/JointGroup API is not yet supported).
* Robot in "Remote/RDK" mode via Flexiv Elements, E-stop within reach.
"""

from __future__ import annotations

import argparse
import sys
import time


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--sn", required=True, help="robot serial number, e.g. Rizon4s-062001")
    ap.add_argument("--gripper", default=None, help="gripper device name to enable (optional)")
    ap.add_argument("--reads", type=int, default=5, help="number of state reads to print")
    ap.add_argument(
        "--allow-motion",
        action="store_true",
        help="exercise a SHORT bounded hold at the current pose (clear workspace first)",
    )
    args = ap.parse_args()

    try:
        import flexivrdk  # noqa: F401
    except Exception as exc:  # pragma: no cover - hardware-only path
        print(f"[FAIL] flexivrdk not importable: {exc}", file=sys.stderr)
        print("       Install with: pip install flexivrdk  (match your robot software)")
        return 2

    print(f"[..] flexivrdk imported (file: {getattr(flexivrdk, '__file__', '?')})")

    from flexiv_control import Robot, RobotConfig
    from flexiv_control.types import ControlMode

    cfg = RobotConfig(backend="flexiv_rdk", robot_sn=args.sn, gripper_name=args.gripper)
    robot = Robot(cfg)

    print(f"[..] connecting to {args.sn} (enable + wait until operational) ...")
    t0 = time.time()
    robot.connect()
    print(f"[OK] operational in {time.time() - t0:.2f}s")

    print(f"[..] reading state x{args.reads} (read-only):")
    for i in range(args.reads):
        s = robot.get_state()
        print(
            f"   [{i}] q={_fmt(s.q)}  tcp_pose={_fmt(s.tcp_pose)}  "
            f"wrench={_fmt(s.wrench)}  grip_w={s.gripper_width:.3f}"
        )
        time.sleep(0.1)

    if args.allow_motion:
        print("[..] --allow-motion: NRT Cartesian impedance hold at CURRENT pose for 2s")
        s = robot.get_state()
        robot.start_cartesian_impedance(realtime=False)
        robot.servo_cartesian_pose(s.tcp_pose, duration=2.0)
        print("[OK] hold complete")
    else:
        print("[i] motion skipped (read-only). Re-run with --allow-motion to test a hold.")

    print("[..] stop + disconnect")
    robot.stop()
    robot.disconnect()
    print("[DONE] smoke test passed")
    return 0


def _fmt(x) -> str:
    try:
        return "[" + ", ".join(f"{v:+.3f}" for v in x) + "]"
    except Exception:
        return str(x)


if __name__ == "__main__":
    raise SystemExit(main())
