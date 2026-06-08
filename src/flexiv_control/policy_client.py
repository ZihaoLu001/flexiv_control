"""Networked chunk-policy client -- the receding-horizon policy seam over HTTP.

A :class:`RemotePolicyClient` turns a remote inference server into a
``policy(obs) -> CartesianChunk | None`` callable for
:class:`~flexiv_control.RecedingHorizonRunner`: it serializes the observation,
POSTs it as JSON, and parses the returned action chunk. This realizes the same
*pattern* openpi/pi0 use (a policy server returns an action chunk from an
observation) -- but over **flexiv_control's own JSON protocol below, NOT the
openpi/pi0 (msgpack/websocket) wire format**. Point it at your own inference
server: a thin shim wrapping an openpi / OpenVLA / diffusion-policy model, or an
MPC solver exposed over HTTP. To talk to a stock openpi server, adapt this
client's encode/decode to that server's wire format.

Protocol (JSON over HTTP POST)::

    request  = {"observation": <RobotState dict>, "instruction": <str | null>}
    response = {"chunk": <CartesianChunk dict>}        # or {"chunk": null} to end

The observation and chunk use the same wire format as the control server
(:func:`flexiv_control.server.protocol.state_to_dict` /
:func:`~flexiv_control.server.protocol.chunk_to_dict`), so a server can build a
chunk with ``CartesianChunk(...)`` and serialize it with the shipped helpers.

Note: the observation here is proprioceptive (:class:`RobotState`); image
observations are not carried yet -- a vision policy server would need camera
frames added to this request.
"""

from __future__ import annotations

import json
import urllib.request
from typing import Optional

from .action_chunk import CartesianChunk
from .server import protocol as P
from .types import RobotState


class RemotePolicyClient:
    """A remote policy server as a ``policy(obs) -> chunk`` callable."""

    def __init__(self, url: str, *, instruction: Optional[str] = None, timeout: float = 30.0):
        self.url = url
        self.instruction = instruction
        self.timeout = float(timeout)

    def infer(self, obs: RobotState) -> Optional[CartesianChunk]:
        payload = json.dumps(
            {"observation": P.state_to_dict(obs), "instruction": self.instruction}
        ).encode("utf-8")
        req = urllib.request.Request(
            self.url, data=payload,
            headers={"Content-Type": "application/json"}, method="POST",
        )
        with urllib.request.urlopen(req, timeout=self.timeout) as resp:
            data = json.loads(resp.read().decode("utf-8"))
        chunk = data.get("chunk")
        if chunk is None:
            return None  # the policy signals "done"
        return P.chunk_from_dict(chunk)

    # usable directly as the RecedingHorizonRunner policy callable
    def __call__(self, obs: RobotState) -> Optional[CartesianChunk]:
        return self.infer(obs)
