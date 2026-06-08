"""RemoteRobot -- the Robot API, over the network.

Same methods as :class:`flexiv_control.Robot`, but every call is an RPC to a
:class:`~flexiv_control.server.server.FlexivControlServer`. This is the seam
that lets an RL trainer or MPC optimizer run on a different machine from the one
holding the arm (Polymetis recommends exactly this), while the controller code
on both sides is identical to the in-process case::

    from flexiv_control import RemoteRobot, CartesianChunk

    with RemoteRobot("192.168.2.100", owner="mpc") as robot:
        robot.start_cartesian_impedance()
        while running:
            s = robot.get_state()
            robot.servo_cartesian_delta(policy(s), duration=0.05)

The client holds the lease for ``owner`` and refreshes it with a background
heartbeat, so if this process dies the arm frees itself for the next user.
Only numpy is required.
"""

from __future__ import annotations

import socket
import threading
import time
from typing import Optional

import numpy as np

from ..action_chunk import CartesianChunk, ExecutionResult, JointChunk
from ..types import GripperCommand, RobotState
from ..server import protocol as P


class RemoteRobotError(RuntimeError):
    pass


class RemoteRobot:
    def __init__(
        self,
        host: str = "127.0.0.1",
        port: int = P.DEFAULT_PORT,
        owner: str = "remote",
        *,
        heartbeat_period: float = 0.5,
        heartbeat_max_failures: int = 3,
        timeout: float = 10.0,
    ):
        self.host = host
        self.port = port
        self.owner = owner
        self.heartbeat_period = heartbeat_period
        self.heartbeat_max_failures = heartbeat_max_failures
        self.timeout = timeout

        self._sock: Optional[socket.socket] = None
        self._rfile = None
        self._wfile = None
        self._io_lock = threading.Lock()
        self._next_id = 0
        self._hb_thread: Optional[threading.Thread] = None
        self._hb_running = False
        self._has_lease = False

    # -- connection ----------------------------------------------------------
    def connect(self) -> "RemoteRobot":
        self._sock = socket.create_connection((self.host, self.port), timeout=self.timeout)
        self._rfile = self._sock.makefile("rb")
        self._wfile = self._sock.makefile("wb")
        self._call("ping")
        return self

    def close(self) -> None:
        self._stop_heartbeat()
        try:
            if self._has_lease:
                self.release_lease()
        except Exception:
            pass
        for f in (self._rfile, self._wfile):
            try:
                if f:
                    f.close()
            except Exception:
                pass
        try:
            if self._sock:
                self._sock.close()
        finally:
            self._sock = None

    def disconnect(self) -> None:
        """Alias for :meth:`close`; matches ``Robot.disconnect()`` so callers are
        API-identical whether they hold a Robot or a RemoteRobot (e.g. the Gym env
        and LeRobot adapter call ``robot.disconnect()``)."""
        self.close()

    def __enter__(self) -> "RemoteRobot":
        self.connect()
        self.acquire_lease()
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    # -- low-level RPC -------------------------------------------------------
    def _call(self, method: str, **params) -> dict:
        if self._wfile is None or self._rfile is None:
            raise RemoteRobotError("not connected; call connect() first")
        with self._io_lock:
            self._next_id += 1
            rid = self._next_id
            self._wfile.write(P.dumps({"id": rid, "method": method, "params": params}))
            self._wfile.flush()
            line = self._rfile.readline()
            if not line:
                raise RemoteRobotError("server closed the connection")
            resp = P.loads(line)
        if not resp.get("ok", False):
            raise RemoteRobotError(resp.get("error", "unknown server error"))
        return resp.get("result", {})

    # -- lease ---------------------------------------------------------------
    def acquire_lease(self, owner: Optional[str] = None, *, force: bool = False) -> None:
        # Accept a positional owner so robot.acquire_lease("mpc") works identically
        # on Robot and RemoteRobot (no more TypeError shims at the call sites).
        if owner is not None:
            self.owner = owner
        self._call("acquire_lease", owner=self.owner, force=force)
        self._has_lease = True
        self._start_heartbeat()

    def release_lease(self) -> None:
        self._stop_heartbeat()
        self._call("release_lease", owner=self.owner)
        self._has_lease = False

    def _start_heartbeat(self) -> None:
        if self._hb_running:
            return
        self._hb_running = True
        self._hb_thread = threading.Thread(target=self._heartbeat, daemon=True)
        self._hb_thread.start()

    def _stop_heartbeat(self) -> None:
        self._hb_running = False
        if self._hb_thread is not None:
            self._hb_thread.join(timeout=1.0)
            self._hb_thread = None

    def _heartbeat(self) -> None:
        # Tolerate transient hiccups: a single slow/dropped heartbeat must not
        # silently let the lease expire under a live client. Give up only after
        # several consecutive failures, or immediately if the server actively
        # rejects us (lease revoked) -- retrying cannot recover that.
        consecutive_failures = 0
        while self._hb_running:
            time.sleep(self.heartbeat_period)
            if not self._hb_running:
                break
            try:
                self._call("heartbeat", owner=self.owner)
                consecutive_failures = 0
            except RemoteRobotError:
                # Server-side rejection (e.g. lease lost/overridden): stop.
                self._has_lease = False
                self._hb_running = False
                break
            except Exception:
                # Transient socket/timeout hiccup: tolerate a few, then give up.
                consecutive_failures += 1
                if consecutive_failures >= self.heartbeat_max_failures:
                    self._has_lease = False
                    self._hb_running = False
                    break

    # -- mirror of the Robot API --------------------------------------------
    def set_safety_profile(self, name: str) -> None:
        self._call("set_safety_profile", owner=self.owner, name=name)

    def get_state(self) -> RobotState:
        return P.state_from_dict(self._call("get_state")["state"])

    def start_cartesian_impedance(
        self,
        *,
        realtime: bool = False,
        stiffness: Optional[np.ndarray] = None,
        damping_ratio: Optional[np.ndarray] = None,
        nullspace_q: Optional[np.ndarray] = None,
    ) -> None:
        params = {"owner": self.owner, "realtime": realtime}
        if stiffness is not None:
            params["stiffness"] = np.asarray(stiffness, float).tolist()
        if damping_ratio is not None:
            params["damping_ratio"] = np.asarray(damping_ratio, float).tolist()
        if nullspace_q is not None:
            params["nullspace_q"] = np.asarray(nullspace_q, float).tolist()
        self._call("start_cartesian_impedance", **params)

    def start_joint_impedance(self, *, realtime: bool = False) -> None:
        self._call("start_joint_impedance", owner=self.owner, realtime=realtime)

    def servo_cartesian_delta(
        self,
        delta,
        *,
        duration: Optional[float] = None,
        frame: str = "base",
        gripper: Optional[GripperCommand] = None,
    ) -> ExecutionResult:
        r = self._call(
            "servo_cartesian_delta",
            owner=self.owner,
            delta=np.asarray(delta, float).tolist(),
            duration=duration,
            frame=frame,
            gripper=P.gripper_to_dict(gripper),
        )
        return P.result_from_dict(r["result"])

    def servo_cartesian_pose(
        self, pose, *, duration: float = 0.2, gripper: Optional[GripperCommand] = None
    ) -> ExecutionResult:
        r = self._call(
            "servo_cartesian_pose",
            owner=self.owner,
            pose=np.asarray(pose, float).tolist(),
            duration=duration,
            gripper=P.gripper_to_dict(gripper),
        )
        return P.result_from_dict(r["result"])

    def execute_cartesian_chunk(
        self, chunk: CartesianChunk, *, blocking: bool = True
    ) -> ExecutionResult:
        # `blocking` is accepted for signature parity with Robot; the server always
        # executes synchronously, so it is a no-op on the wire.
        r = self._call(
            "execute_cartesian_chunk", owner=self.owner, chunk=P.chunk_to_dict(chunk)
        )
        return P.result_from_dict(r["result"])

    def execute_joint_chunk(self, chunk: JointChunk) -> ExecutionResult:
        r = self._call(
            "execute_joint_chunk", owner=self.owner, chunk=P.joint_chunk_to_dict(chunk)
        )
        return P.result_from_dict(r["result"])

    def move_joint(
        self, q_target, *, duration: float = 3.0, realtime: bool = False
    ) -> ExecutionResult:
        r = self._call(
            "move_joint",
            owner=self.owner,
            q=np.asarray(q_target, float).tolist(),
            duration=duration,
            realtime=realtime,
        )
        return P.result_from_dict(r["result"])

    def command_gripper(self, cmd: GripperCommand) -> None:
        self._call("command_gripper", owner=self.owner, gripper=P.gripper_to_dict(cmd))

    def home(self) -> None:
        self._call("home", owner=self.owner)

    def stop(self) -> None:
        self._call("stop", owner=self.owner)
