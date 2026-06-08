"""Receding-horizon execution: predict a chunk, execute H_exec, replan.

The canonical action-chunking loop (Diffusion Policy / openpi / ACT): a policy
maps an observation to an action *chunk*; the robot executes only the first
``horizon_exec`` waypoints, then re-observes and re-plans. The ``policy`` is any
callable ``RobotState -> CartesianChunk | None`` -- it is the **seam between a VLA
/ planner / MPC and the controller**. It can wrap:

* an openpi / OpenVLA websocket *policy server* (``infer(obs) -> action chunk``),
* an MPC solver returning the first slice of its horizon,
* a local sampler + simulated-future ranker (ActAhead),

and ``None`` ends the run. Set ``n_execute`` on the returned chunk to control the
execution horizon (Diffusion-Policy reference: predict ~16, execute ~8).
"""

from __future__ import annotations

import time
from typing import Callable, Optional

from .action_chunk import CartesianChunk, ExecutionResult
from .types import RobotState

Policy = Callable[[RobotState], Optional[CartesianChunk]]
OnStep = Callable[[int, CartesianChunk, ExecutionResult], Optional[bool]]


class RecedingHorizonRunner:
    """Drive a robot with a policy in a receding-horizon loop.

    ``robot`` is anything with ``get_state()`` and ``execute_cartesian_chunk()``
    (a :class:`~flexiv_control.Robot` or a
    :class:`~flexiv_control.RemoteRobot`).
    """

    def __init__(self, robot, policy: Policy, *, max_steps: Optional[int] = None):
        self.robot = robot
        self.policy = policy
        self.max_steps = max_steps

    def run(self, *, on_step: Optional[OnStep] = None) -> int:
        """Blocking loop: observe -> policy -> execute the chunk's first
        ``horizon_exec`` waypoints -> repeat. Returns the number of executed
        chunks; stops when the policy returns ``None``, ``on_step`` returns
        ``False``, or ``max_steps`` is reached."""
        steps = 0
        while self.max_steps is None or steps < self.max_steps:
            obs = self.robot.get_state()
            chunk = self.policy(obs)
            if chunk is None:
                break
            result = self.robot.execute_cartesian_chunk(chunk)
            steps += 1
            if on_step is not None and on_step(steps, chunk, result) is False:
                break
        return steps

    def run_streaming(self, loop, *, replan_hz: float = 5.0) -> int:
        """Infer-ahead loop over a :class:`ReactiveServoLoop`: enqueue the next
        chunk while the loop streams the current one (real-time chunking). The
        loop is the single writer and holds-on-stale if the policy stalls.

        ``loop`` must already be started. Returns the number of chunks enqueued.
        """
        steps = 0
        period = 1.0 / float(replan_hz)
        next_t = time.perf_counter()
        while self.max_steps is None or steps < self.max_steps:
            obs = loop.get_state()
            chunk = self.policy(obs)
            if chunk is None:
                break
            loop.enqueue_chunk(chunk)
            steps += 1
            next_t += period
            sl = next_t - time.perf_counter()
            if sl > 0:
                time.sleep(sl)
        return steps
