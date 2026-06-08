"""Host-wide single-owner lock for a robot.

The in-process :class:`~flexiv_control.server.lease.Lease` arbitrates the clients
of ONE server process. This adds a *host-wide* advisory lock so two independent
OS processes on the same machine cannot both command the same arm -- e.g. two
server instances, or a server plus a direct ``Robot`` script. It is backed by an
atomic lock FILE holding the owner's PID; a holder whose PID is dead (crashed) is
automatically reclaimable.

Cross-platform: no ``fcntl``/``flock`` dependency -- it uses an atomic write plus
PID-liveness, which is sufficient for advisory single-owner arbitration in a lab.
"""

from __future__ import annotations

import json
import os
import tempfile
import time
from typing import Optional


def _pid_alive(pid: int) -> bool:
    if pid is None or pid <= 0:
        return False
    try:
        if os.name == "nt":  # pragma: no cover - platform-specific
            import ctypes

            PROCESS_QUERY_LIMITED_INFORMATION = 0x1000
            h = ctypes.windll.kernel32.OpenProcess(
                PROCESS_QUERY_LIMITED_INFORMATION, False, int(pid)
            )
            if not h:
                return False
            ctypes.windll.kernel32.CloseHandle(h)
            return True
        os.kill(int(pid), 0)
        return True
    except Exception:
        return False


class HostLockError(RuntimeError):
    pass


class HostLock:
    """Advisory host-wide lock for ``robot_id``, backed by a PID lock file.

    The lock is held while the owning process is alive; a crashed owner's lock is
    reclaimable (its PID is dead). Set ``FLEXIV_CONTROL_LOCK_DIR`` to choose where
    the lock file lives (defaults to the system temp dir).
    """

    def __init__(self, robot_id: str = "rizon", *, lock_dir: Optional[str] = None, owner: str = ""):
        self.robot_id = robot_id
        self.owner = owner
        d = lock_dir or os.environ.get("FLEXIV_CONTROL_LOCK_DIR") or tempfile.gettempdir()
        self.path = os.path.join(d, f"flexiv_control_{robot_id}.lock")
        self._held = False

    # -- internals ----------------------------------------------------------
    def _read(self) -> Optional[dict]:
        try:
            with open(self.path) as f:
                return json.load(f)
        except Exception:
            return None

    def _write_atomic(self, info: dict) -> None:
        tmp = f"{self.path}.{os.getpid()}.tmp"
        with open(tmp, "w") as f:
            json.dump(info, f)
        os.replace(tmp, self.path)  # atomic on POSIX and Windows

    def _live_other_holder(self) -> Optional[dict]:
        """The lock file's contents iff held by a *different, alive* process."""
        info = self._read()
        if not info:
            return None
        pid = int(info.get("pid", -1))
        if pid == os.getpid():
            return None
        return info if _pid_alive(pid) else None

    # -- API ----------------------------------------------------------------
    def is_locked_by_other(self) -> bool:
        return self._live_other_holder() is not None

    def holder(self) -> Optional[dict]:
        return self._live_other_holder()

    def acquire(self, owner: Optional[str] = None, *, force: bool = False) -> None:
        if owner is not None:
            self.owner = owner
        other = self._live_other_holder()
        if other is not None and not force:
            raise HostLockError(
                f"robot {self.robot_id!r} is host-locked by pid {other.get('pid')} "
                f"(owner {other.get('owner')!r}); another process holds it. Pass "
                f"force=True to override, or stop that process."
            )
        self._write_atomic({"pid": os.getpid(), "owner": self.owner, "stamp": time.time()})
        self._held = True

    def release(self) -> None:
        if not self._held:
            return
        info = self._read()
        if info and int(info.get("pid", -1)) == os.getpid():
            try:
                os.remove(self.path)
            except Exception:
                pass
        self._held = False

    def __enter__(self) -> "HostLock":
        self.acquire()
        return self

    def __exit__(self, *exc) -> None:
        self.release()
