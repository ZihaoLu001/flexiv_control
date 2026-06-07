#!/usr/bin/env python3
"""05 - SpaceMouse teleoperation.

Teleop runs a reactive servo loop: every tick it reads the SpaceMouse and
streams a Cartesian delta to the robot at ``control_hz``, and the safety
watchdog holds position if input goes stale. The SpaceMouse is also the same
intervention source RL uses (example 03), and for an always-on single-writer
streaming variant (the Python analogue of the C++ 1 kHz daemon) see
``flexiv_control.server.ReactiveServoLoop``.

    python examples/05_spacemouse_teleop.py            # scripted motion, no device
    python examples/05_spacemouse_teleop.py --device   # real SpaceMouse (needs extras)

Install the device driver with::

    pip install "flexiv-control[teleop]"      # pyspacemouse / hidapi

The scripted source traces a small circle so the example runs with no hardware
at all (fake backend + scripted input).
"""

import argparse

from flexiv_control import Robot, RobotConfig
from flexiv_control.teleop import (
    PySpaceMouseSource,
    ScriptedSpaceMouseSource,
    SpaceMouseTeleop,
)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", action="store_true", help="use a real SpaceMouse")
    parser.add_argument("--backend", default="fake", help="fake | flexiv_rdk")
    parser.add_argument("--seconds", type=float, default=3.0)
    args = parser.parse_args()

    robot = Robot(RobotConfig(backend=args.backend, control_hz=200.0))

    if args.device:
        source = PySpaceMouseSource()       # raises if driver/device missing
        print("using a real SpaceMouse; move it to jog the arm.")
    else:
        source = ScriptedSpaceMouseSource()  # hardware-free circle
        print("no device: tracing a scripted circle on the fake backend.")

    with robot:
        teleop = SpaceMouseTeleop(
            robot=robot,
            source=source,
            pos_scale=0.05,     # m/s at full deflection
            rot_scale=0.6,      # rad/s at full deflection
        )
        # run() acquires the lease, starts Cartesian impedance, then reads the
        # source and streams a Cartesian delta every tick until the time budget
        # is up. The safety watchdog holds position if input goes stale.
        teleop.run(duration=args.seconds)

    print("teleop finished.")


if __name__ == "__main__":
    main()
