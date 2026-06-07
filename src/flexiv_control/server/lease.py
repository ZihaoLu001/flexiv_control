"""A single-writer lease for the robot.

In a shared lab the failure mode is two processes commanding the arm at once --
an RL trainer and a SpaceMouse, or two students' MPC nodes. That is dangerous,
not just messy. The server owns exactly one :class:`Lease`; a client must hold
it to issue any motion command. A lease has an owner string and an expiry: if
the holder dies without releasing, the lease frees itself after
``ttl_seconds`` so the arm is not bricked for the next person. Clients refresh
it with a cheap heartbeat.

This is intentionally simple (one process, in-memory). It is the cross-process
guarantee the single-process ``Robot.acquire_lease`` cannot make on its own.
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from typing import Optional


class LeaseError(RuntimeError):
    pass


@dataclass
class LeaseInfo:
    owner: str
    acquired_at: float
    expires_at: float


class Lease:
    def __init__(self, ttl_seconds: float = 2.0):
        self.ttl = float(ttl_seconds)
        self._lock = threading.Lock()
        self._info: Optional[LeaseInfo] = None

    # -- queries -------------------------------------------------------------
    @property
    def owner(self) -> str:
        self._expire_if_stale()
        with self._lock:
            return self._info.owner if self._info else ""

    def info(self) -> Optional[LeaseInfo]:
        self._expire_if_stale()
        with self._lock:
            return self._info

    def _expire_if_stale(self) -> None:
        with self._lock:
            if self._info and time.time() > self._info.expires_at:
                self._info = None

    # -- mutations -----------------------------------------------------------
    def acquire(self, owner: str, *, force: bool = False) -> LeaseInfo:
        if not owner:
            raise LeaseError("lease owner must be a non-empty string")
        self._expire_if_stale()
        with self._lock:
            if self._info and self._info.owner != owner and not force:
                raise LeaseError(
                    f"robot is leased by {self._info.owner!r}; "
                    f"acquire with force=True to override"
                )
            now = time.time()
            self._info = LeaseInfo(owner=owner, acquired_at=now, expires_at=now + self.ttl)
            return self._info

    def heartbeat(self, owner: str) -> LeaseInfo:
        with self._lock:
            if not self._info or self._info.owner != owner:
                raise LeaseError(f"{owner!r} does not hold the lease")
            self._info.expires_at = time.time() + self.ttl
            return self._info

    def release(self, owner: str) -> None:
        with self._lock:
            if self._info and self._info.owner != owner:
                raise LeaseError(f"{owner!r} cannot release lease held by {self._info.owner!r}")
            self._info = None

    def check(self, owner: str) -> None:
        """Raise unless ``owner`` currently holds a non-expired lease."""
        self._expire_if_stale()
        with self._lock:
            if not self._info:
                raise LeaseError("no lease held; call acquire_lease first")
            if self._info.owner != owner:
                raise LeaseError(
                    f"command from {owner!r} but lease is held by {self._info.owner!r}"
                )
