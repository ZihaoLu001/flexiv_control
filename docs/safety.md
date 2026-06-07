# Safety

> Community project, **NOT affiliated with Flexiv Robotics**.

> **The shipped numbers are a starting template, not a guarantee.** Workspace
> bounds in particular are specific to *your* cell. Validate every profile on
> your own robot at low speed before trusting it. This software does not replace
> the robot's own safety-rated stop, your risk assessment, or an E-stop within
> reach.

Safety is a first-class citizen here, not an afterthought, because this
controller is meant for a **shared** lab and the wider community — "everyone be
careful" does not scale. Every Cartesian and joint setpoint passes through a
cheap per-tick filter before it reaches the robot, and the filter always
**reports** what it did.

## Two pieces: a profile and a filter

- **`SafetyProfile`** — a named, version-controlled set of limits. Loaded from a
  YAML file so an experiment is reproducible: "this run used `tabletop_safe`."
- **`SafetyFilter`** — a pure-numpy, microsecond-cost guard that runs inside the
  control loop (cheap enough for 1 kHz). For each setpoint it either lightly
  **clips** the command back into bounds or **rejects** it (→ hold position),
  and records the outcome.

```python
from flexiv_control import load_safety_profile, SafetyProfile, SafetyFilter
profile = load_safety_profile("tabletop_safe")   # by name (shipped) or file path
```

You normally never touch these directly — the `Robot` facade loads the profile
named in your `RobotConfig.default_safety_profile` and applies the filter for
you. Switch profiles at runtime with `robot.set_safety_profile("free_space_fast")`.

## What the filter enforces

| Guard | Behaviour | Reported `stop_reason` |
|---|---|---|
| Workspace box (`ws_x/ws_y/ws_z`) | TCP position **clipped** back into the box | `workspace_limit` |
| TCP linear/angular speed | per-tick step **clipped** to `max_linear_speed` / `max_angular_speed` | `tcp_speed_limit` |
| Pose jump | step larger than `max_pose_jump_*` per tick **rejected** → hold | `pose_jump_limit` |
| Joint position | target **clipped** to `[lower+margin, upper-margin]` | `joint_limit` |
| Joint speed | **clipped** to `max_joint_speed_scale ×` nominal limit | `joint_limit` |
| Contact wrench | measured wrench over `max_contact_wrench` → **stop** | `contact_wrench` |
| Command watchdog | no fresh command within `command_timeout_ms` → **hold** | `stale_command` |
| State watchdog | no fresh state within `state_timeout_ms` → **stop** | `stale_state` |

The filter holds the **previous commanded setpoint** (not live state) as its
reference, so limits are enforced on the command stream and don't fight normal
tracking lag. The control loop calls `filter.reset(state)` at the start of every
motion so the first command is anchored to the robot's current pose. The
interpolator is **velocity-aware** and time-stretches a too-fast segment so it
still reaches the waypoint instead of being clipped short — so a speed cap slows
a motion down rather than truncating it. See
[action_contract.md](action_contract.md) and
[architecture.md](architecture.md).

## The four shipped profiles

Under `configs/safety/`:

| Profile | For | Character |
|---|---|---|
| `tabletop_safe` | the conservative default; shared tabletop manipulation | tight box, 0.25 m/s, 40 N contact stop |
| `free_space_fast` | fast free-space reaching, no expected contact | larger box, higher speed, lower contact tolerance |
| `rl_conservative` | RL data collection / rollouts (the Gym env default) | small box, low speed, low contact threshold — fail safe under a random policy |
| `contact_manipulation` | pushing, insertion, wiping | allows higher contact wrench, force-control friendly |

## Profile YAML schema

A profile file looks like this (`tabletop_safe.yaml`):

```yaml
name: tabletop_safe

workspace:                      # axis-aligned TCP box, base frame, metres
  x: [0.25, 0.75]
  y: [-0.45, 0.45]
  z: [0.06, 0.70]

tcp_limits:
  max_linear_speed: 0.25        # m/s
  max_angular_speed: 0.60       # rad/s
  max_pose_jump_linear: 0.03    # m   per control tick
  max_pose_jump_angular: 0.15   # rad per control tick

joint_limits:
  margin_rad: 0.08              # shrink the hard limits by this
  max_joint_speed_scale: 0.30   # fraction of nominal joint-velocity limit
  # optional: lower: [...7...]  upper: [...7...]   (defaults to Rizon 4/4s nominal)

contact:
  max_wrench: [40, 40, 40, 5, 5, 5]   # [fx,fy,fz,tx,ty,tz] -> hard stop if exceeded

watchdog:
  command_timeout_ms: 100
  state_timeout_ms: 20
  stop_on_stale_command: true
```

Any omitted field falls back to the dataclass default in
`flexiv_control.safety.SafetyProfile`.

## Tuning for your cell

1. **Measure your workspace box first.** Jog the arm to the corners of the
   region you actually want to allow and read `robot.get_state().tcp_position`.
   Set `workspace` a few centimetres *inside* that. The default box is a
   placeholder.
2. **Start slow.** Keep `max_linear_speed` low (≤ 0.1 m/s) for first runs, raise
   it once motions look right.
3. **Set the contact stop below what hurts.** For tabletop work 40 N is a sane
   ceiling; lower it for delicate setups. For deliberate contact tasks use
   `contact_manipulation` and raise it knowingly.
4. **Copy, don't edit in place.** Make `configs/safety/mylab_tabletop.yaml`,
   commit it, and reference it by path or name. Point `FLEXIV_CONTROL_CONFIGS`
   at your own config directory to override the shipped ones without touching
   this repo.
5. **Watch the report.** If `result.clipped` is true or `result.stop_reason` is
   not `"none"`, the filter intervened — inspect before raising limits.

## What this is *not*

The software filter is a behavioural guardrail layered on top of — never a
replacement for — the robot's safety-rated functions, a proper risk assessment,
and a physical E-stop. Torque-level mode (`RT_JOINT_TORQUE`) is the highest-risk
path and is off unless a lab admin explicitly enables it with a dedicated
profile; it is never a default entry point for RL or a high-level planner.
