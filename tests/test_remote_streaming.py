"""Server-side single-writer streaming (start_servo_loop / servo_stream) and the
networked policy-server client (RemotePolicyClient) driving a receding horizon."""

from __future__ import annotations

import http.server
import json
import threading
import time

import numpy as np
import pytest

from flexiv_control import (
    CartesianChunk,
    RecedingHorizonRunner,
    RemotePolicyClient,
    Robot,
    RobotConfig,
)
from flexiv_control.client import RemoteRobot
from flexiv_control.client.remote_robot import RemoteRobotError
from flexiv_control.server import FlexivControlServer
from flexiv_control.server import protocol as P


def test_server_side_servo_loop_streams_holds_and_blocks():
    srv = FlexivControlServer(config=RobotConfig(backend="fake"), port=8811, lease_ttl=10.0)
    srv.serve_in_thread()
    time.sleep(0.3)
    try:
        rr = RemoteRobot("127.0.0.1", 8811, owner="t")
        rr.connect()
        rr.acquire_lease("t")
        rr.start_cartesian_impedance()
        s0 = rr.get_state()

        rr.start_servo_loop(control_hz=200.0)
        tgt = s0.tcp_pose.copy()
        tgt[0] += 0.04
        for _ in range(25):  # stream the target fire-and-forget
            rr.servo_stream(tgt)
            time.sleep(0.03)
        moved = rr.get_state()
        assert moved.tcp_pose[0] - s0.tcp_pose[0] > 0.02  # the loop drove the arm

        # stop streaming -> the loop HOLDS (watchdog), no drift
        held = rr.get_state().tcp_pose.copy()
        time.sleep(0.4)
        assert np.linalg.norm(rr.get_state().tcp_pose[:3] - held[:3]) < 0.01

        # blocking motion RPCs are rejected while the single-writer loop owns the arm
        with pytest.raises(RemoteRobotError):
            rr.execute_cartesian_chunk(
                CartesianChunk.from_waypoint_array([[0.5, 0.0, 0.30, 1.0, 20]])
            )

        # stop the loop -> blocking motion works again
        rr.stop_servo_loop()
        p = rr.get_state().tcp_pose
        rr.execute_cartesian_chunk(
            CartesianChunk.from_waypoint_array([[p[0], p[1], p[2], 1.0, 10]],
                                               safety_profile="free_space_fast")
        )
        rr.disconnect()
    finally:
        srv.shutdown()


def test_remote_policy_client_drives_receding_horizon():
    calls = {"n": 0}

    class Handler(http.server.BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def do_POST(self):
            n = int(self.headers["Content-Length"])
            body = json.loads(self.rfile.read(n))
            obs = body["observation"]
            calls["n"] += 1
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            if calls["n"] > 3:  # signal "done"
                self.wfile.write(json.dumps({"chunk": None}).encode())
                return
            x, y, z = obs["tcp_pose"][0], obs["tcp_pose"][1], obs["tcp_pose"][2]
            chunk = CartesianChunk.from_pose_array(
                np.array([[x + 0.02, y, z, 1, 0, 0, 0, 1, 40]]),
                safety_profile="free_space_fast",
            )
            self.wfile.write(json.dumps({"chunk": P.chunk_to_dict(chunk)}).encode())

    httpd = http.server.HTTPServer(("127.0.0.1", 8822), Handler)
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    try:
        r = Robot(RobotConfig(backend="fake"))
        r.connect()
        r.start_cartesian_impedance()
        s0 = r.get_state()
        client = RemotePolicyClient("http://127.0.0.1:8822/infer")
        n = RecedingHorizonRunner(r, client, max_steps=10).run()
        s1 = r.get_state()
        assert n == 3              # ran until the server returned chunk=null
        assert calls["n"] == 4
        assert s1.tcp_pose[0] - s0.tcp_pose[0] > 0.04  # policy advanced the arm
        r.disconnect()
    finally:
        httpd.shutdown()
