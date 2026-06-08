"""The control server: one owner of the robot, reachable over the network.

Mirrors the architecture every serious research controller converged on
(Polymetis' controller-manager server, SERL's robot server, frankapy's
interface process): a single process owns the hardware and runs the control
loop; clients -- an RL trainer, an MPC node, a teleop bridge, possibly on other
machines -- talk to it over a thin protocol. That is what lets the heavy,
non-real-time learning/planning code live wherever it likes (even a different
box, as Polymetis explicitly recommends) while one well-behaved process keeps
the arm safe.

Transport here is the stdlib ``socketserver`` + newline-delimited JSON, so the
client needs nothing but numpy. The valuable parts -- single ownership, the
lease, the safety filter on every setpoint -- are transport-independent.

Every motion/mode RPC carries an ``owner`` and is checked against the
:class:`~flexiv_control.server.lease.Lease`; backend access is serialized by a
lock so two threads can never interleave on one arm.

Two execution models: (1) **synchronous** motion RPCs (servo_*/execute_*/move_*)
run the interpolate/stream loop to completion inside the handler; (2) an
**always-on single-writer streaming mode** -- ``start_servo_loop`` spins up a
:class:`~flexiv_control.server.control_loop.ReactiveServoLoop` as the sole backend
writer, and ``servo_stream`` publishes the latest target fire-and-forget, so a
stalled remote client **holds** (the loop's watchdog re-issues the measured pose)
rather than gapping the stream -- the Deoxys / Polymetis / SERL single-writer
pattern. While the loop runs, blocking motion RPCs are rejected (single writer).
"""

from __future__ import annotations

import socketserver
import threading
from typing import Any, Callable, Dict, Optional

import numpy as np

from ..config import RobotConfig
from ..robot import Robot
from ..types import GripperCommand, ImpedanceParams
from . import protocol as P
from .control_loop import ReactiveServoLoop
from .host_lock import HostLock
from .lease import Lease, LeaseError

# Blocking motion RPCs that must not run while the single-writer servo loop owns
# the backend (rejected in _dispatch when a loop is active).
_LOOP_BLOCKED = frozenset({
    "servo_cartesian_delta", "servo_cartesian_pose", "execute_cartesian_chunk",
    "execute_joint_chunk", "move_joint", "home",
})


class FlexivControlServer:
    def __init__(
        self,
        robot: Optional[Robot] = None,
        config: Optional[RobotConfig] = None,
        host: str = "0.0.0.0",
        port: int = P.DEFAULT_PORT,
        lease_ttl: float = 2.0,
        host_lock: bool = True,
    ):
        self.robot = robot or Robot(config or RobotConfig())
        self.host = host
        self.port = port
        self.lease = Lease(ttl_seconds=lease_ttl)
        # Host-wide single-owner lock (across OS processes), in addition to the
        # in-process client Lease. None to disable (e.g. tests / multi-arm hosts).
        self._host_lock = (
            HostLock(self.robot.cfg.robot_id, owner="server") if host_lock else None
        )
        self._robot_lock = threading.Lock()
        self._servo_loop: Optional[ReactiveServoLoop] = None  # single-writer streaming
        self._tcp: Optional[socketserver.ThreadingTCPServer] = None
        self._handlers: Dict[str, Callable[[dict], Any]] = self._build_handlers()

    # -- lifecycle -----------------------------------------------------------
    def start(self) -> None:
        # Host-wide arbitration: refuse to start if another *live* process holds
        # this robot (a second server, or a direct script). Reclaimed automatically
        # if the previous holder crashed (its PID is dead).
        if self._host_lock is not None:
            self._host_lock.acquire()
        self.robot.connect()
        server = self
        # Seed lease auto-acquire off (server is the source of truth).
        self.robot._owner = None  # noqa: SLF001

        class Handler(socketserver.StreamRequestHandler):
            def handle(self) -> None:
                try:
                    for line in self.rfile:
                        line = line.strip()
                        if not line:
                            continue
                        try:
                            req = P.loads(line)
                        except Exception as e:  # malformed line
                            self.wfile.write(
                                P.dumps({"id": -1, "ok": False, "error": f"bad json: {e}"})
                            )
                            continue
                        resp = server._dispatch(req)
                        self.wfile.write(P.dumps(resp))
                except OSError:
                    # Client disconnected mid-request/response (BrokenPipeError,
                    # ConnectionResetError, ...). End this connection quietly
                    # instead of dumping a traceback via socketserver.handle_error.
                    return

        socketserver.ThreadingTCPServer.allow_reuse_address = True
        self._tcp = socketserver.ThreadingTCPServer((self.host, self.port), Handler)
        self._tcp.daemon_threads = True

    def serve_forever(self) -> None:
        if self._tcp is None:
            self.start()
        assert self._tcp is not None
        try:
            self._tcp.serve_forever()
        finally:
            self.shutdown()

    def serve_in_thread(self) -> threading.Thread:
        if self._tcp is None:
            self.start()
        assert self._tcp is not None
        t = threading.Thread(target=self._tcp.serve_forever, daemon=True)
        t.start()
        return t

    def shutdown(self) -> None:
        if self._servo_loop is not None:
            self._servo_loop.stop()
            self._servo_loop = None
        if self._tcp is not None:
            self._tcp.shutdown()
            self._tcp.server_close()
            self._tcp = None
        try:
            self.robot.stop()
        finally:
            self.robot.disconnect()
            if self._host_lock is not None:
                self._host_lock.release()

    # -- dispatch ------------------------------------------------------------
    def _dispatch(self, req: dict) -> dict:
        rid = req.get("id", -1)
        method = req.get("method", "")
        params = req.get("params", {}) or {}
        if self._servo_loop is not None and method in _LOOP_BLOCKED:
            return {"id": rid, "ok": False, "error":
                    "servo loop active (single-writer streaming); call "
                    "stop_servo_loop before blocking motion commands"}
        handler = self._handlers.get(method)
        if handler is None:
            return {"id": rid, "ok": False, "error": f"unknown method: {method!r}"}
        try:
            result = handler(params)
            return {"id": rid, "ok": True, "result": result}
        except LeaseError as e:
            return {"id": rid, "ok": False, "error": f"lease: {e}"}
        except Exception as e:  # surface backend / validation errors to the client
            return {"id": rid, "ok": False, "error": f"{type(e).__name__}: {e}"}

    def _require_lease(self, params: dict) -> str:
        owner = params.get("owner", "")
        self.lease.check(owner)
        self.robot._owner = owner  # noqa: SLF001  keep the facade's check consistent
        return owner

    def _build_handlers(self) -> Dict[str, Callable[[dict], Any]]:
        return {
            "ping": lambda p: {"pong": True},
            "acquire_lease": self._h_acquire_lease,
            "release_lease": self._h_release_lease,
            "heartbeat": self._h_heartbeat,
            "get_lease": lambda p: {"owner": self.lease.owner},
            "set_safety_profile": self._h_set_safety_profile,
            "get_state": self._h_get_state,
            "start_cartesian_impedance": self._h_start_cartesian_impedance,
            "start_joint_impedance": self._h_start_joint_impedance,
            "servo_cartesian_delta": self._h_servo_cartesian_delta,
            "servo_cartesian_pose": self._h_servo_cartesian_pose,
            "execute_cartesian_chunk": self._h_execute_cartesian_chunk,
            "execute_joint_chunk": self._h_execute_joint_chunk,
            "move_joint": self._h_move_joint,
            "command_gripper": self._h_command_gripper,
            "home": self._h_home,
            "stop": self._h_stop,
            "start_servo_loop": self._h_start_servo_loop,
            "servo_stream": self._h_servo_stream,
            "servo_stream_joint": self._h_servo_stream_joint,
            "stop_servo_loop": self._h_stop_servo_loop,
        }

    # -- handlers ------------------------------------------------------------
    def _h_acquire_lease(self, p: dict) -> dict:
        info = self.lease.acquire(p.get("owner", ""), force=bool(p.get("force", False)))
        return {"owner": info.owner, "expires_at": info.expires_at}

    def _h_release_lease(self, p: dict) -> dict:
        self.lease.release(p.get("owner", ""))
        with self._robot_lock:
            self.robot.stop()
        return {"released": True}

    def _h_heartbeat(self, p: dict) -> dict:
        info = self.lease.heartbeat(p.get("owner", ""))
        return {"expires_at": info.expires_at}

    def _h_set_safety_profile(self, p: dict) -> dict:
        self._require_lease(p)
        with self._robot_lock:
            self.robot.set_safety_profile(p["name"])
        return {"profile": self.robot.profile.name}

    def _h_get_state(self, p: dict) -> dict:
        loop = self._servo_loop
        if loop is not None:
            s = loop.get_state()  # carries loop jitter/period/hold status
        else:
            with self._robot_lock:
                s = self.robot.get_state()
        s.active_owner = self.lease.owner
        return {"state": P.state_to_dict(s)}

    def _h_start_cartesian_impedance(self, p: dict) -> dict:
        self._require_lease(p)
        imp = None
        if "stiffness" in p:
            imp = ImpedanceParams(
                stiffness=np.asarray(p["stiffness"], float),
                damping_ratio=np.asarray(
                    p.get("damping_ratio", [0.7] * 6), float
                ),
            )
        ns = None if p.get("nullspace_q") is None else np.asarray(p["nullspace_q"], float)
        with self._robot_lock:
            self.robot.start_cartesian_impedance(
                impedance=imp, realtime=bool(p.get("realtime", False)), nullspace_q=ns
            )
        return {"started": True}

    def _h_start_joint_impedance(self, p: dict) -> dict:
        self._require_lease(p)
        with self._robot_lock:
            self.robot.start_joint_impedance(realtime=bool(p.get("realtime", False)))
        return {"started": True}

    def _h_servo_cartesian_delta(self, p: dict) -> dict:
        self._require_lease(p)
        with self._robot_lock:
            r = self.robot.servo_cartesian_delta(
                np.asarray(p["delta"], float),
                duration=p.get("duration"),
                frame=p.get("frame", "base"),
                gripper=P.gripper_from_dict(p.get("gripper")),
            )
        return {"result": P.result_to_dict(r)}

    def _h_servo_cartesian_pose(self, p: dict) -> dict:
        self._require_lease(p)
        with self._robot_lock:
            r = self.robot.servo_cartesian_pose(
                np.asarray(p["pose"], float),
                duration=float(p.get("duration", 0.2)),
                gripper=P.gripper_from_dict(p.get("gripper")),
            )
        return {"result": P.result_to_dict(r)}

    def _h_execute_cartesian_chunk(self, p: dict) -> dict:
        self._require_lease(p)
        chunk = P.chunk_from_dict(p["chunk"])
        with self._robot_lock:
            r = self.robot.execute_cartesian_chunk(chunk, blocking=True)
        return {"result": P.result_to_dict(r)}

    def _h_execute_joint_chunk(self, p: dict) -> dict:
        self._require_lease(p)
        chunk = P.joint_chunk_from_dict(p["chunk"])
        with self._robot_lock:
            r = self.robot.execute_joint_chunk(chunk)
        return {"result": P.result_to_dict(r)}

    def _h_move_joint(self, p: dict) -> dict:
        self._require_lease(p)
        with self._robot_lock:
            r = self.robot.move_joint(
                np.asarray(p["q"], float),
                duration=float(p.get("duration", 3.0)),
                realtime=bool(p.get("realtime", False)),
            )
        return {"result": P.result_to_dict(r)}

    def _h_command_gripper(self, p: dict) -> dict:
        self._require_lease(p)
        g = P.gripper_from_dict(p["gripper"]) or GripperCommand()
        with self._robot_lock:
            self.robot.command_gripper(g)
        return {"ok": True}

    def _h_home(self, p: dict) -> dict:
        self._require_lease(p)
        with self._robot_lock:
            self.robot.home()
        return {"ok": True}

    def _h_stop(self, p: dict) -> dict:
        # stop does not require the lease -- anyone may e-stop.
        if self._servo_loop is not None:
            self._servo_loop.stop()
            self._servo_loop = None
        with self._robot_lock:
            self.robot.stop()
        return {"stopped": True}

    # -- always-on single-writer streaming loop (hold-on-stale) -------------
    def _h_start_servo_loop(self, p: dict) -> dict:
        self._require_lease(p)
        with self._robot_lock:
            if self._servo_loop is None:
                loop = ReactiveServoLoop(self.robot, control_hz=p.get("control_hz"))
                loop.start()
                self._servo_loop = loop
        return {"started": True}

    def _h_servo_stream(self, p: dict) -> dict:
        self._require_lease(p)
        if self._servo_loop is None:
            raise RuntimeError("no servo loop running; call start_servo_loop first")
        self._servo_loop.set_cartesian_target(
            np.asarray(p["pose"], float), gripper=P.gripper_from_dict(p.get("gripper"))
        )
        return {"ok": True}

    def _h_servo_stream_joint(self, p: dict) -> dict:
        self._require_lease(p)
        if self._servo_loop is None:
            raise RuntimeError("no servo loop running; call start_servo_loop first")
        self._servo_loop.set_joint_target(np.asarray(p["q"], float))
        return {"ok": True}

    def _h_stop_servo_loop(self, p: dict) -> dict:
        self._require_lease(p)
        loop, self._servo_loop = self._servo_loop, None
        if loop is not None:
            loop.stop()
        return {"stopped": True}
