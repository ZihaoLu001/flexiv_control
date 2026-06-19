# Versions & what to verify

> Community project, **NOT affiliated with Flexiv Robotics**.

The Flexiv RDK API has changed across releases — method names, mode enums, and
binding details differ between versions, and RDK is **version-matched to robot
firmware**. `flexiv_control` isolates all of that behind one backend file and
marks every version-sensitive call so you can check it against *your* install
instead of trusting a hard-coded guess.

## Which RDK generation this targets

The Python backend (`backends/flexiv_rdk.py`) targets **RDK v1.x** (robot
software v3.x — e.g. lab firmware v3.9 ↔ RDK v1.x). There are **three
incompatible RDK API generations**, so confirm which one you have before first
use:

| Generation | Constructor | State read | Command form |
|---|---|---|---|
| **v0.x** (legacy) | `Robot(robot_ip, local_ip)` | `getRobotStates(out)`, **camelCase** fields (`tcpPose`, `extWrenchInBase`) | flat `setMode()` |
| **v1.x** (this backend) | `Robot(robot_sn)` | `states()` returns one struct, **snake_case** (`tcp_pose`, `ext_wrench_in_world`) | flat `SwitchMode` / `Send*` / `Stream*` |
| **v2.x** (newest) | `Robot(robot_sn)` | `states()` returns a **dict keyed by `JointGroup`** | commands wrapped in `*Cmd` objects, setters take a leading `group` arg |

This backend assumes the **v1.x** column: `Robot(robot_sn)`, `states().tcp_pose`,
`SendCartesianMotionForce(pose, ...)`, `SetCartesianImpedance(K_x, Z_x)`,
`SendJointPosition(positions, velocities, accelerations, max_vel, max_acc)`,
`SetForceControlFrame(CoordType)`, gripper `Enable(name)` → `Init()` → `Move(width, velocity, force_limit)`.
It will **not** work as-is on v0.x (different constructor + camelCase) or v2.x
(dict/`JointGroup` API). If your robot ships v2.x, the changes are confined to
this one backend file. Don't upgrade RDK without the matched firmware.

### Hardware-verified on flexivrdk **1.7** (Rizon4s-062626) — the exact gotchas

These four were confirmed (and fixed) against a live robot; bake them in if you port:
- **`Mode` enum has only `IDLE` + `NRT_*` + `UNKNOWN`** — there are **no `RT_*` members** in
  1.7. Building a `ControlMode→Mode` map that names `Mode.RT_JOINT_POSITION` etc.
  `AttributeError`s; guard each entry by `hasattr`.
- **`SendJointPosition` takes FIVE `list[float]`**: `(target_pos, target_vel, target_acc,
  max_vel, max_acc)`. Passing four (dropping `target_acc`) `TypeError`s. Pass plain
  `float`s — `np.float64` is rejected.
- **Force-control modes need the F/T sensor zeroed first**: before `SwitchMode(
  NRT_CARTESIAN_MOTION_FORCE)`, run the `ZeroFTSensor` primitive in
  `NRT_PRIMITIVE_EXECUTION` (arm at rest), else event **301004** faults the switch. The
  backend does this once in `connect()`.
- **Gripper**: `Gripper.Enable(device_name)` (the name from Flexiv Elements → Settings →
  Device, e.g. `Flexiv-GN01`) **before** `Init()`. `Init()` alone → "No gripper enabled".

## The `# VERIFY:` convention

Anywhere the code calls an RDK symbol whose exact name/signature may differ in
your version, the line is tagged with a `# VERIFY:` comment. Before first
hardware use, grep for them and confirm each against your installed RDK:

```bash
grep -rn "VERIFY" src/flexiv_control/backends/flexiv_rdk.py cpp/ src/flexiv_control/adapters/
```

They cluster in three places:

- `src/flexiv_control/backends/flexiv_rdk.py` — the Python real-robot backend
  (mode enums, `SendCartesianMotionForce` / `SendJointPosition`, gripper calls,
  state field names).
- `cpp/src/rt_server.cpp` — the Tier-B daemon (`flexiv::rdk::Robot`,
  `flexiv::rdk::Scheduler`, `StreamCartesianMotionForce`, the CMake target name
  `flexiv::flexiv_rdk`).
- `src/flexiv_control/adapters/lerobot_robot.py` — the LeRobot feature-dict
  schema (LeRobot's API has also moved between versions).

The contract, safety, interpolation, server, client, and Gym env above the
backend line are pure Python+numpy and are **not** version-sensitive.

## Check your installed versions

```python
import flexivrdk
print(getattr(flexivrdk, "__version__", "see Flexiv docs / package metadata"))
```

```bash
python -c "import numpy, sys; print('python', sys.version.split()[0], '| numpy', numpy.__version__)"
pip show flexiv-control | grep -i version
```

If you use the optional extras, also note their versions (`gymnasium`,
`lerobot`, `mujoco`) — those ecosystems move quickly too.

## Things that are easy to get wrong

- **Quaternion order is `(w, x, y, z)`.** RDK's TCP pose is
  `[x, y, z, qw, qx, qy, qz]`; this library keeps `(w, x, y, z)` everywhere to
  match. If you bridge to a library that uses `(x, y, z, w)` (e.g. some ROS or
  SciPy paths), convert at the boundary.
- **Mode names.** The `ControlMode` enum mirrors RDK's modes but the exact RDK
  spelling can differ by version — confirm the NRT/RT motion-force and joint
  modes you use.
- **NRT vs RT and the license.** RT streaming modes need the **Professional**
  license (and, in C++, a real-time scheduler/kernel + root). NRT works on
  **Standard**. See [flexiv_setup.md](flexiv_setup.md).
- **Gripper API.** Gripper init/move/grasp calls and the gripper name string
  vary with hardware and RDK version — verify against your gripper.

## Pinning guidance

- **`flexiv_control` itself**: pin the exact version you validated against in
  downstream projects (e.g. `flexiv-control==0.1.1`) so an upgrade is a
  deliberate, reviewed step. The current release is `0.1.1`.
- **RDK / firmware**: pin to the matched pair Flexiv documents for your robot;
  do not upgrade one without the other.
- **numpy**: the core needs only numpy and is written to work across modern
  numpy (tested on 1.x and 2.x). Pin if your wider environment is sensitive.
- **Optional extras**: pin `gymnasium`, `lerobot`, and `mujoco` to versions you
  have validated, since their APIs change between minor releases.

## When something breaks after an RDK upgrade

1. Re-grep the `# VERIFY:` lines and diff against the new RDK API.
2. Run the hardware-free suite first to prove the version-independent core is
   still green:
   ```bash
   pip install -e ".[dev]"
   pytest -q
   ```
   Then re-run the [first-run checklist](flexiv_setup.md) on hardware at low
   speed.
3. Fix only the backend file(s); nothing above the backend line should need to
   change.
