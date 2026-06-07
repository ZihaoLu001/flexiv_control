<div align="center">

<img src="docs/assets/banner.svg" alt="flexiv_control" width="100%"/>

<h1>flexiv_control</h1>

<p><b>A unified, real-time execution &amp; safety layer for Flexiv Rizon arms.</b><br/>
One action contract for teleoperation, MPC, reinforcement learning, and real-to-sim-to-real manipulation.</p>

<p>
<img src="https://img.shields.io/badge/license-Apache--2.0-19E3C4?style=flat-square" alt="license"/>
<img src="https://img.shields.io/badge/python-3.8%2B-4DA3FF?style=flat-square&logo=python&logoColor=white" alt="python"/>
<img src="https://img.shields.io/github/actions/workflow/status/ZihaoLu001/flexiv_control/ci.yml?branch=main&style=flat-square&label=CI&color=19E3C4" alt="ci"/>
<img src="https://img.shields.io/badge/tests-38%20passing-2ea043?style=flat-square" alt="tests"/>
<img src="https://img.shields.io/badge/core_deps-numpy-4DA3FF?style=flat-square" alt="deps"/>
<img src="https://img.shields.io/badge/status-alpha-f0a020?style=flat-square" alt="status"/>
</p>

<p><sub>⚠️ Community project — <b>NOT affiliated with, endorsed by, or supported by Flexiv Robotics.</b>
"Flexiv", "Rizon", and "RDK" are trademarks of their owner.</sub></p>

<p>
<a href="https://zihaolu001.github.io/flexiv_control/"><b>🌐 Live page</b></a> ·
<a href="#quick-start-no-hardware-needed"><b>Quick start</b></a> ·
<a href="#documentation"><b>Docs</b></a> ·
<a href="#two-tiers-of-real-time"><b>Architecture</b></a> ·
<a href="docs/integration_planner.md"><b>Planner integration</b></a> ·
<a href="docs/safety.md"><b>Safety</b></a>
</p>

</div>

---

`flexiv_control` is a thin, fast layer between a high-level decision maker (a
policy, an MPC planner, a teleoperator) and a Flexiv Rizon arm. Its one job:
take a high-level action, turn it into a **safe** stream of setpoints at the
control rate, and report back what actually happened.

Every consumer — a high-level planner, a lab mate's RL code, a teleop session,
the wider community — speaks the **same action contract**, and every backend
(a real Rizon via Flexiv RDK, a dependency-free `fake` backend, or MuJoCo)
consumes the same filtered setpoint stream. That is what makes one controller
reusable across projects without forks.

> **Why it looks the way it does.** This shape — *a thin Python client over a
> real-time-capable control loop, one action interface, and sim + real behind a
> single backend switch* — is what Deoxys, Polymetis, frankapy, SERL/HIL-SERL,
> and LeRobot all converged on. The [design rationale](docs/design_rationale.md)
> walks through the field survey and the decisions (and gives a critical look at
> a ROS-2-first alternative).

## ✦ Highlights

- **One action contract** for everything: `CartesianChunk`, `CartesianDelta`,
  `JointChunk`, `GripperCommand`, and a quantified `ExecutionResult`.
  `CartesianChunk.from_waypoint_array(u)` ingests a planner's `(H,5)` action
  array directly.
- **Safety is first-class**: a named, version-controlled `SafetyProfile` + a
  microsecond per-tick `SafetyFilter` (workspace box, speed/jump caps, joint
  limits, contact-wrench stop, watchdog) that *clips or rejects and reports*.
- **Two tiers of real-time, same contract** — start on Tier A, upgrade only if a
  measurement says you must (see below).
- **Backend-agnostic**: develop offline on `fake`, simulate on `mujoco`, deploy
  on a real Rizon — policy code unchanged.
- **Optional, not required**: a cross-process server + `RemoteRobot` client
  (numpy-only, no ROS), a Gymnasium env, a LeRobot adapter, a SpaceMouse teleop /
  RL-intervention helper, a C++ 1 kHz daemon, and a ROS 2 overlay.
- **Core depends on numpy only.** Everything else is an optional extra.

## ⚡ Quick start (no hardware needed)

```bash
git clone https://github.com/ZihaoLu001/flexiv_control.git
cd flexiv_control
pip install -e .          # core: numpy only
```

```python
from flexiv_control import Robot, RobotConfig, CartesianChunk

robot = Robot(RobotConfig(backend="fake"))      # dependency-free simulation
robot.connect()
robot.start_cartesian_impedance()

chunk = CartesianChunk.from_waypoint_array([[0.45, 0.0, 0.30, 1.0, 20],
                                            [0.50, 0.0, 0.25, 0.0, 20]])
result = robot.execute_cartesian_chunk(chunk)
print(result.success, result.path_tracking_error)

robot.disconnect()
```

Or try the CLI and the examples:

```bash
flexiv-control demo                 # offline FakeBackend demo, no hardware
python examples/01_fake_hello.py
```

<details>
<summary><b>Install options</b> (RL, teleop, LeRobot, MuJoCo, real hardware, dev)</summary>

```bash
pip install -e .                    # core (numpy only)
pip install -e ".[rl]"              # + Gymnasium env
pip install -e ".[teleop]"          # + SpaceMouse
pip install -e ".[lerobot]"         # + LeRobot data/training/viz
pip install -e ".[mujoco]"          # + MuJoCo backend
pip install -e ".[flexiv]"          # + flexivrdk (real hardware; pin to your robot's version)
pip install -e ".[dev]"             # + pytest, ruff
```
</details>

## ⛓ Two tiers of real-time

The decisive hardware fact: **the Rizon runs its hard real-time impedance/motion
loop *inside* the robot controller.** The host streams setpoints; the robot does
the 1 kHz servoing. That gives two tiers that share everything above the backend:

| | **Tier A — Python (default)** | **Tier B — C++ 1 kHz daemon (optional)** |
|---|---|---|
| Loop | 100–500 Hz host loop, RDK **non-real-time** modes | true 1 kHz host loop, RDK **real-time** modes |
| Needs | nothing special | `PREEMPT_RT` kernel, root, `flexiv::rdk::Scheduler` |
| License | **Standard** | **Professional** |
| Good for | planner chunks, MPC, RL | high-rate streaming MPC, tight contact, torque research |

Both speak the **same action contract and the same wire protocol**, so the
Python client / Gym env / ROS overlay do not change when you move from A to B.
Start on Tier A. See [docs/architecture.md](docs/architecture.md) and
[`cpp/README.md`](cpp/README.md).

## ◳ The contract is the spine

```
 Language policy   MPC planner   RL trainer   SpaceMouse
        \              |             |            /
         └──►  CartesianChunk / CartesianDelta / JointChunk  ◄──┘
                              │
            Robot facade → SafetyFilter (per-tick) → Interpolator → backend → Rizon
```

Because the contract is the only thing crossing the boundary, the network
server, Gym env, and ROS 2 overlay are all pure pass-through. Full reference:
[docs/action_contract.md](docs/action_contract.md).

## 📚 Documentation

| Doc | What |
|---|---|
| [architecture.md](docs/architecture.md) | the stack, the two RT tiers, components, boundaries |
| [design_rationale.md](docs/design_rationale.md) | field survey, decisions, critique of the ROS-2-first alternative |
| [action_contract.md](docs/action_contract.md) | the contract objects, conventions, `from_waypoint_array` |
| [safety.md](docs/safety.md) | profiles, the filter, the four shipped profiles, tuning |
| [flexiv_setup.md](docs/flexiv_setup.md) | bringing up a real Rizon, licenses, first-run checklist |
| [versions.md](docs/versions.md) | RDK version sensitivity and the `# VERIFY:` markers |
| [integration_planner.md](docs/integration_planner.md) | receding-horizon planners: replan loop, real2sim2real, failure signal |
| [integration_mpc.md](docs/integration_mpc.md) | MPC / high-rate closed loops, `ReactiveServoLoop` |
| [integration_rl.md](docs/integration_rl.md) | Gymnasium env, HIL-SERL intervention, sim→real |
| [integration_teleop.md](docs/integration_teleop.md) | SpaceMouse teleop and the MoveIt-Servo bridge |

## ▶ Examples

| File | Shows |
|---|---|
| `examples/01_fake_hello.py` | connect, read state, run a chunk on `fake` |
| `examples/02_cartesian_chunk.py` | the action contract and `ExecutionResult` |
| `examples/03_rl_gym_env.py` | Gymnasium env + HIL-SERL intervention |
| `examples/04_mpc_loop.py` | a high-rate closed loop |
| `examples/05_spacemouse_teleop.py` | teleop (scripted, or `--device`) |
| `examples/06_lerobot_record.py` | recording in the LeRobot dataset format |

## ⌗ Repository layout

```
src/flexiv_control/      core library (contract, safety, interpolation, robot facade, backends)
  server/  client/       optional cross-process server + RemoteRobot
  envs/    adapters/      Gymnasium env + LeRobot adapter
  teleop/                 SpaceMouse teleop / RL intervention
configs/                 robot + safety + control YAMLs (edit for your cell)
cpp/                     optional Tier-B 1 kHz RT daemon
ros2/                    optional ROS 2 overlay (msgs + bringup, MoveIt-Servo jog)
docs/                    the docs above (+ the GitHub Pages landing page)
examples/                runnable, hardware-free examples
tests/                   pytest suite
```

## 🛠 Development

```bash
pip install -e ".[dev]"
ruff check src tests examples
pytest -q
```

CI runs the suite on Python 3.8 / 3.10 / 3.12 and separately verifies the
numpy-only core install (`.github/workflows/ci.yml`).

## ⏿ Status &amp; contributing

Alpha (`0.1.0`). The Python core, safety, contract, server/client, Gym env,
teleop, and the offline path are tested; the real-hardware (`flexiv_rdk`),
MuJoCo, LeRobot, and C++/ROS paths have `# VERIFY:` markers to confirm against
your installed versions before first use ([docs/versions.md](docs/versions.md)).
Contributions welcome — see [CONTRIBUTING.md](CONTRIBUTING.md).

> **Before any motion on real hardware:** keep an E-stop within reach and
> validate your `SafetyProfile` (especially the workspace box) on your own cell
> at low speed. See [docs/safety.md](docs/safety.md).

## 📄 License

Apache-2.0 — see [LICENSE](LICENSE). If you use this in academic work, see
[CITATION.cff](CITATION.cff).

<div align="center"><sub>Built for the Flexiv Rizon · community-maintained · not affiliated with Flexiv Robotics</sub></div>
