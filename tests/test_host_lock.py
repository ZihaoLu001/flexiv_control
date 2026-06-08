"""Tests for the host-wide single-owner lock (cross-OS-process arbitration)."""

from __future__ import annotations

import os
import subprocess
import sys
import time

import pytest

from flexiv_control.server.host_lock import HostLock, HostLockError


def test_basic_acquire_release(tmp_path):
    lk = HostLock("testrobot", lock_dir=str(tmp_path))
    assert not lk.is_locked_by_other()
    lk.acquire("a")
    assert os.path.exists(lk.path)
    # same process -> a second lock object is NOT treated as "other" (same PID)
    lk2 = HostLock("testrobot", lock_dir=str(tmp_path))
    assert not lk2.is_locked_by_other()
    lk2.acquire("b")  # must not raise (same host process)
    lk.release()


def test_blocks_other_live_process_and_reclaims_on_death(tmp_path):
    ready = tmp_path / "ready"
    code = (
        "import time\n"
        "from flexiv_control.server.host_lock import HostLock\n"
        f"HostLock('xrobot', lock_dir={str(tmp_path)!r}).acquire('child')\n"
        f"open({str(ready)!r}, 'w').close()\n"
        "time.sleep(30)\n"
    )
    proc = subprocess.Popen([sys.executable, "-c", code], env=dict(os.environ))
    try:
        for _ in range(200):
            if ready.exists():
                break
            time.sleep(0.05)
        assert ready.exists(), "child did not acquire the lock"

        lk = HostLock("xrobot", lock_dir=str(tmp_path))
        assert lk.is_locked_by_other()
        with pytest.raises(HostLockError):
            lk.acquire("parent")          # blocked by the live child
        lk.acquire("parent", force=True)  # force overrides
    finally:
        proc.terminate()
        proc.wait(timeout=5)

    # child is dead -> its lock is reclaimable without force
    lk2 = HostLock("xrobot", lock_dir=str(tmp_path))
    assert not lk2.is_locked_by_other()
    lk2.acquire("parent2")  # must not raise
    lk2.release()
