# The action contract

> Community project, **NOT affiliated with Flexiv Robotics**.

The action contract is the spine of `flexiv_control`. Every high-level component
— a receding-horizon planner, an MPC horizon, an RL policy, a SpaceMouse bridge —
emits one of a tiny set of objects, and every backend (real Rizon, `fake`,
MuJoCo) knows how to execute them. Nobody re-invents "how do I talk to the
robot," which is exactly what lets one controller serve many projects.

All of these live in `flexiv_control.action_chunk` and are re-exported from the
top level:

```python
from flexiv_control import (
    CartesianChunk, CartesianWaypoint, CartesianDelta,
    JointChunk, JointWaypoint, GripperCommand, ExecutionResult,
)
```

## Conventions (read this once)

These hold everywhere in the library, end to end:

- **Units are SI**: metres, radians, seconds, Newtons, Newton-metres.
- **Quaternions are `(w, x, y, z)`** — the same order Flexiv RDK uses for TCP
  pose `[x, y, z, qw, qx, qy, qz]`. One convention end to end avoids a whole
  class of silent sign/order bugs.
- **A TCP pose is a length-7 array** `[x, y, z, qw, qx, qy, qz]` in the robot
  **base** frame by default. Frames are named strings (`"base"`, `"tcp"`,
  `"flange"`, `"world"`); the default is `"base"`.
- **`n_frames` and `duration` are interchangeable.** A waypoint may specify
  either. `n_frames` (a per-waypoint frame count) is converted to seconds at the active
  control rate: `duration = n_frames / control_hz`. The same chunk therefore
  runs identically whether the loop is 100 Hz or 1 kHz, as long as `n_frames`
  is interpreted at that rate.

## `CartesianWaypoint` — one Cartesian target

```python
CartesianWaypoint(
    position,              # required, length-3 [x, y, z]
    quaternion=None,       # (w, x, y, z); None -> HOLD previous/current orientation
    gripper=None,          # GripperCommand; None -> HOLD gripper
    n_frames=None,         # frame count (int > 0)           } give exactly
    duration=None,         # seconds (alternative to n_frames) } one of these
    frame="base",
)
```

`quaternion=None` means *hold orientation*, which is exactly what a
position-only planner (one emitting `(x, y, z)` waypoints) wants — it maps in with
zero changes. A waypoint must specify either `n_frames` or `duration`.

## `CartesianChunk` — the core action type

A short, bounded sequence of waypoints. Used by a receding-horizon planner (execute the first
segment, then replan), MPC (the first slice of a horizon), scripted
manipulation, and high-level RL.

```python
CartesianChunk(
    waypoints,                       # List[CartesianWaypoint], >= 1
    impedance=ImpedanceParams(),     # compliance for the whole chunk
    force_control=None,              # ForceControlParams or None
    max_tcp_linear_speed=0.25,       # m/s     } kinematic envelope enforced by
    max_tcp_angular_speed=0.60,      # rad/s   } the interpolator + safety filter
    max_tcp_linear_acc=1.0,          # m/s^2
    max_tcp_angular_acc=2.0,         # rad/s^2
    max_contact_wrench=None,         # [fx,fy,fz,tx,ty,tz]; None -> profile default
    safety_profile="tabletop_safe",  # named profile loaded before executing
    frame="base",
)
```

Helpful attributes: `chunk.horizon` (number of waypoints) and
`chunk.total_duration(control_hz)`.

### `CartesianChunk.from_waypoint_array(u)` — ingest a planner's action array

A common receding-horizon / action-chunking action has the form `u = ((x_j, y_j, z_j, w_j, n_j))_{j=1..H}`:
a short sequence of Cartesian positions, each with a normalised gripper command
`w_j ∈ [0, 1]` (1 = open, 0 = closed) and an integer number of low-level control
frames `n_j`. This constructor turns that `(H, 5)` array straight into a chunk:

```python
import numpy as np
from flexiv_control import CartesianChunk

u = np.array([
    [0.45, 0.00, 0.30, 1.0, 20],   # reach, gripper open, 20 frames
    [0.50, 0.00, 0.22, 1.0, 15],   # descend
    [0.50, 0.00, 0.22, 0.0, 10],   # close gripper
])
chunk = CartesianChunk.from_waypoint_array(u)            # orientation held throughout
```

The normalised gripper maps to a width with `width = clip(w, 0, 1) * 0.08`
(≈ the 80 mm stroke of the lab's gripper). Override `gripper_force=` and pass any
`CartesianChunk` field through `**chunk_kwargs` (e.g.
`safety_profile="contact_manipulation"`). `from_waypoint_array` is position-only,
so orientation is always held; build full-SE(3) `CartesianWaypoint`s if you need
to command orientation.

## `CartesianDelta` — the RL / MPC / teleop workhorse

A *relative* end-effector move integrated on top of the current pose. This is
the standard RL/MPC/teleop action and maps 1:1 onto robosuite/MuJoCo OSC-style
actions, which is what keeps sim→real transfer clean.

```python
CartesianDelta(
    delta,             # length-6 [dx, dy, dz, droll, dpitch, dyaw]
    gripper=None,
    duration=0.05,     # 20 Hz default control step
    frame="base",
)
```

You rarely build this by hand — `Robot.servo_cartesian_delta([...])` and the Gym
env do it for you.

## Joint space — `JointWaypoint` / `JointChunk`

For resets, homing, and executing a planned joint trajectory (e.g. from MoveIt):

```python
JointWaypoint(positions, n_frames=None, duration=None)   # give one of the two

JointChunk(
    waypoints,                      # List[JointWaypoint], >= 1
    max_joint_speed_scale=0.3,      # fraction of nominal joint-velocity limits
    safety_profile="tabletop_safe",
)
```

## `GripperCommand`

```python
GripperCommand(
    width=0.0,        # metres
    force=20.0,       # Newtons (clamping force)
    velocity=0.1,     # m/s
    grasp=False,      # True -> move-until-contact; False -> position move
)
```

## `ExecutionResult` — quantifies the "execution" failure bucket

Every blocking execution call returns this. A receding-horizon planner's failure taxonomy typically has an
"execution" category; this object turns that category into something observable
and quantifiable rather than a guess. The planner can attribute a bad outcome to
execution vs perception/ranking *with evidence*.

```python
result = robot.execute_cartesian_chunk(chunk)
result.success              # bool
result.clipped              # was any setpoint modified to stay in bounds?
result.stop_reason          # "none" | "workspace_limit" | "contact_wrench" | ...
result.executed_duration    # seconds actually run
result.path_tracking_error  # max ||pose_cmd - pose_meas|| over the run (m)
result.max_tcp_speed        # m/s
result.max_joint_speed      # rad/s
result.max_wrench           # N
result.gripper_width_final  # m
result.final_state          # RobotState snapshot at the end
result.log                  # dict of extra diagnostics
```

A planner that logs `clipped` / `stop_reason` / `path_tracking_error` per chunk
gets a clean signal for *why* a rollout failed — see
[integration_planner.md](integration_planner.md).

## How each consumer uses the contract

| Consumer | Emits | Path |
|---|---|---|
| Receding-horizon planner | `CartesianChunk.from_waypoint_array(u)` | execute first segment → replan |
| MPC loop | `CartesianDelta` per tick, or first slice of a horizon as a `CartesianChunk` | [integration_mpc.md](integration_mpc.md) |
| RL policy | `[dx,dy,dz,droll,dpitch,dyaw,gripper]` → `CartesianDelta` (via `FlexivRealEnv`) | [integration_rl.md](integration_rl.md) |
| SpaceMouse teleop / RL intervention | `CartesianDelta` per tick | [integration_teleop.md](integration_teleop.md) |
| Reset / home / MoveIt plan | `JointChunk` | `Robot.move_joint` / `execute_joint_chunk` |

Because the contract is the only thing crossing the boundary, the network server
([architecture.md](architecture.md)), the Gym env, and the ROS 2 overlay are all
pure pass-through: they serialize and forward these exact objects.
