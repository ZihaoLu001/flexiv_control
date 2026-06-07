# Tier-B: the optional C++ 1 kHz real-time daemon

**Community project, NOT affiliated with Flexiv Robotics.**

You almost certainly do **not** need this to start. The Python stack (Tier A)
drives the Rizon at 100–500 Hz through the RDK's *non-real-time* modes, and the
robot runs its hard real-time impedance loop **internally**, so chunk execution,
MPC, and RL are all reactive and safe from Python alone. Build this daemon only
when you genuinely need a **true 1 kHz host loop**:

- high-rate streaming MPC where you want to send a fresh Cartesian setpoint
  every millisecond,
- tight contact / force tasks that benefit from host-side 1 kHz updates,
- torque-level research (extend this daemon; it ships Cartesian-only by design).

`rt_server` speaks the **same newline-delimited-JSON wire protocol** as the
Python `FlexivControlServer`, so your existing Python client, Gym env, and ROS 2
overlay talk to it unchanged — only the executor underneath is different.

## What it does

```
[Python client / MPC / RL] --JSON/TCP--> [network thread] --mailbox--> [RT thread @1kHz] --RDK--> [Rizon]
```

- A **network thread** accepts one client, parses `set_cartesian_target` /
  `get_state` / `stop`, and publishes the latest setpoint into a lock-light
  double-buffered mailbox.
- A **real-time thread**, scheduled by `flexiv::rdk::Scheduler` at 1 kHz, reads
  the latest setpoint, applies a cheap workspace + per-tick speed clamp (mirror
  of `flexiv_control/safety.py`), and streams it with
  `StreamCartesianMotionForce`. It never allocates and never blocks on the
  network; if commands go stale (>100 ms) it holds the last commanded pose.

## Prerequisites

1. **Flexiv RDK**, version-matched to your robot's software. Install per Flexiv's
   instructions and either install its CMake config or point `CMAKE_PREFIX_PATH`
   at the install tree.
2. **A real-time-capable kernel** for genuine 1 kHz: a `PREEMPT_RT` patched
   kernel (best) or at least a `lowlatency` kernel. Without it you can still run,
   but expect jitter and occasional missed ticks.
3. **`nlohmann/json`** (header-only): `sudo apt install nlohmann-json3-dev`, or
   vendor the single header to `cpp/third_party/nlohmann/json.hpp`.
4. **A Professional RDK license** is required for the RT streaming modes; the
   Standard license only exposes the NRT modes (which is exactly what Tier A
   uses, so Tier A works without the Pro license).
5. **Root**, because the scheduler sets real-time thread priority and CPU
   affinity.

## Build

```bash
cd cpp
mkdir -p build && cd build
cmake -DCMAKE_PREFIX_PATH=/path/to/flexiv_rdk/install ..
make -j
```

## Run

```bash
# default 1 kHz loop on TCP port 8766
sudo ./build/rt_server <ROBOT_SERIAL> --port 8766 --freq 1000
```

Then from Python, point a `RemoteRobot` at it exactly as you would the Python
server. (Today the daemon implements the streaming subset —
`set_cartesian_target`, `get_state`, `stop`. The full method set / a thin
client shim for the rest is on the roadmap; the protocol is identical, so it is
purely additive.)

## ⚠️ Version-sensitive API

RDK class/enum/method names have changed across versions. Every such call in
`src/rt_server.cpp` is flagged with a `// VERIFY:` comment — most importantly:

- header path & namespace (`flexiv::rdk::Robot` vs older `flexiv::Robot`),
- the `flexiv::rdk::Mode::RT_CARTESIAN_MOTION_FORCE` enum path,
- `robot.states()` field names (`tcp_pose`, `q`, `ext_wrench_in_world`),
- `Scheduler::AddTask(callback, name, interval_ticks, priority[, cpu])`,
- `StreamCartesianMotionForce(pose, wrench)` argument shapes,
- the CMake imported target name (`flexiv::flexiv_rdk`).

Check each against the RDK you link before trusting the binary on hardware.

## Safety

This daemon enforces a workspace box and a per-tick linear-speed cap in the RT
thread, and backs off to a held pose on stale commands. It is **not** a
substitute for the robot's own safety configuration: keep the Rizon's safety
settings, contact-wrench ceilings, and an E-stop within reach. Validate in a
clear workspace, at low speed, with a finger on the stop, before any autonomous
run.
