"""``flexiv-control`` command-line entry point.

Subcommands::

    flexiv-control serve  --backend fake --port 8766     # run the control server
    flexiv-control home   --backend fake                 # send the arm home
    flexiv-control state  --backend fake                 # print one RobotState
    flexiv-control demo                                  # offline FakeBackend demo

Nothing here needs hardware unless you pass ``--backend flexiv_rdk``.
"""

from __future__ import annotations

import argparse
import sys

import numpy as np

from . import __version__
from .config import RobotConfig
from .robot import Robot


def _robot(args) -> Robot:
    cfg = (
        RobotConfig.load(args.config)
        if getattr(args, "config", None)
        else RobotConfig(backend=args.backend, robot_sn=getattr(args, "robot_sn", "") or "")
    )
    if getattr(args, "backend", None):
        cfg.backend = args.backend
    return Robot(cfg)


def _cmd_serve(args) -> int:
    from .server.server import FlexivControlServer

    server = FlexivControlServer(
        robot=_robot(args), host=args.host, port=args.port, lease_ttl=args.lease_ttl
    )
    print(f"[flexiv-control] serving backend={server.robot.cfg.backend!r} "
          f"on {args.host}:{args.port} (ctrl-c to stop)")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n[flexiv-control] shutting down")
    return 0


def _cmd_home(args) -> int:
    robot = _robot(args)
    with robot:
        robot.acquire_lease("cli")
        robot.home()
        print("[flexiv-control] homed:", np.round(robot.get_state().q, 3))
    return 0


def _cmd_state(args) -> int:
    robot = _robot(args)
    with robot:
        s = robot.get_state()
        print("mode        :", s.control_mode.value)
        print("q           :", np.round(s.q, 3))
        print("tcp_pose    :", np.round(s.tcp_pose, 4))
        print("wrench      :", np.round(s.wrench, 2))
        print("gripper     :", round(s.gripper_width, 4))
    return 0


def _cmd_demo(args) -> int:  # noqa: ARG001
    from .action_chunk import CartesianChunk

    robot = Robot(RobotConfig(backend="fake"))
    with robot:
        robot.acquire_lease("demo")
        robot.start_cartesian_impedance()
        u = [[0.45, 0.0, 0.30, 1.0, 20],
             [0.50, 0.05, 0.25, 0.0, 20],
             [0.45, 0.0, 0.30, 1.0, 20]]
        result = robot.execute_cartesian_chunk(CartesianChunk.from_waypoint_array(u))
        print("[flexiv-control] demo chunk executed:")
        print("  success            :", result.success)
        print("  clipped            :", result.clipped)
        print("  path_tracking_error:", round(result.path_tracking_error, 5))
        print("  final tcp position :", np.round(result.final_state.tcp_position, 4))
    return 0


def main(argv=None) -> int:
    p = argparse.ArgumentParser(prog="flexiv-control", description=__doc__)
    p.add_argument("--version", action="version", version=f"flexiv-control {__version__}")
    sub = p.add_subparsers(dest="cmd", required=True)

    common = argparse.ArgumentParser(add_help=False)
    common.add_argument("--backend", default="fake",
                        help="fake | flexiv_rdk | mujoco (default: fake)")
    common.add_argument("--config", default=None, help="path or name of a robot config YAML")
    common.add_argument("--robot-sn", dest="robot_sn", default="",
                        help="Flexiv robot serial number (flexiv_rdk backend)")

    s = sub.add_parser("serve", parents=[common], help="run the control server")
    s.add_argument("--host", default="0.0.0.0")
    s.add_argument("--port", type=int, default=8766)
    s.add_argument("--lease-ttl", type=float, default=2.0)
    s.set_defaults(func=_cmd_serve)

    h = sub.add_parser("home", parents=[common], help="send the arm to its home posture")
    h.set_defaults(func=_cmd_home)

    st = sub.add_parser("state", parents=[common], help="print one RobotState and exit")
    st.set_defaults(func=_cmd_state)

    d = sub.add_parser("demo", help="offline FakeBackend demo (no hardware)")
    d.set_defaults(func=_cmd_demo)

    args = p.parse_args(argv)
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
