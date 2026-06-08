# Integration: reinforcement learning

> Community project, **NOT affiliated with Flexiv Robotics**.

`FlexivRealEnv` exposes the arm as a standard **Gymnasium** environment, so any
RL or imitation-learning stack (SERL/HIL-SERL, LeRobot, stable-baselines3, your
own actor–learner) drives the robot through the same safe, leased control path
as everything else. The key property for sim→real: **the same action contract
drives sim and real**, through the same safety filter and backend. See
`examples/03_rl_gym_env.py`.

## The environment

```python
from flexiv_control import RobotConfig
from flexiv_control.envs import make_env       # or: from flexiv_control import FlexivRealEnv

env = make_env(
    config=RobotConfig(backend="fake"),        # "mujoco" for sim, "rizon4s_lab" for real
    control_hz=20.0,
    safety_profile="rl_conservative",           # applied automatically
    pos_scale=0.05, rot_scale=0.20,             # action scaling (m, rad)
    max_episode_steps=200,
    reward_fn=my_reward,                        # (obs, action) -> (reward, done, info)
)
obs, info = env.reset()
for _ in range(200):
    action = policy(obs)                        # 7-dim, in [-1, 1]
    obs, reward, terminated, truncated, info = env.step(action)
    if terminated or truncated:
        break
env.close()
```

Spaces:

```
action      = [dx, dy, dz, drx, dry, drz, gripper]   in [-1, 1]   (shape (7,); rotation = axis-angle rotvec)
observation = [q(7), dq(7), tcp_pose(7), wrench(6), gripper_width(1)]   (28-dim)
```

`gripper ∈ [-1, 1]` maps to open/close; positional axes are scaled by
`pos_scale`/`rot_scale` into a `CartesianDelta` and pushed through the safety
filter, so a random or early-training policy is bounded by the profile rather
than by luck. Gymnasium is optional — without it the env falls back to a tiny
`Env`/`Box` shim so it still imports and runs in CI; install the real thing with
`pip install "flexiv-control[rl]"`.

## Sim → real with no policy change

Point the env at a different backend; the policy and reward are untouched:

- `backend="fake"` — dependency-free, for offline development and CI.
- `backend="mujoco"` — your simulation scene (wire the `MujocoBackend` seam to
  your model).
- `backend="flexiv_rdk"` — the real arm. Use the `rl_conservative` profile
  (small box, low speed, low contact threshold) for data collection.

Because the action is the standard low-dim Cartesian-delta used by
robosuite/SERL/LeRobot, a policy trained in sim transfers to real through the
identical interface. ([action_contract.md](action_contract.md))

## HIL-SERL: SpaceMouse intervention

The SpaceMouse is both a teleop device and an RL **intervention** device — a
human can overwrite the policy action mid-episode, and those corrections are
exactly what HIL-SERL learns from. The teleop helper exposes this directly:

```python
from flexiv_control.teleop import SpaceMouseTeleop, ScriptedSpaceMouseSource

teleop = SpaceMouseTeleop(robot=env.robot, source=ScriptedSpaceMouseSource())
# PySpaceMouseSource() for a real device

obs, _ = env.reset()
for _ in range(200):
    a_policy = policy(obs)
    a_exec, intervened = teleop.intervention(a_policy)   # human overrides if deadman held
    obs, r, term, trunc, info = env.step(a_exec)
    buffer.add(obs, a_exec, r, intervened)               # flag teaches HIL-SERL
```

`intervention(policy_action)` returns `(action, intervened)`: when the operator's
deadman button is held and the device is moved, the human delta replaces the
policy action and `intervened=True`. See [integration_teleop.md](integration_teleop.md).

## Recording datasets (LeRobot)

For collecting demonstrations / logging episodes, use the LeRobot adapter — it
exposes the LeRobot `Robot` surface with the same Cartesian-delta action as the
env, so a policy trained in `FlexivRealEnv` records and replays unchanged.
Building a `LeRobotDataset` from the frames is a thin caller-side step against
your installed `lerobot` (its dataset API varies by release). See
`examples/06_lerobot_record.py`; install with `pip install "flexiv-control[lerobot]"`.

## Cross-machine training

The GPU box is rarely the robot box. Run the server next to the arm and give the
env a `RemoteRobot`:

```python
from flexiv_control import RemoteRobot
from flexiv_control.envs import make_env

r = RemoteRobot("ROBOT_HOST_IP", 8766, owner="rl_env"); r.connect(); r.acquire_lease()
env = make_env(robot=r, safety_profile="rl_conservative")
```

The lease guarantees a single writer to the arm, so a crashed trainer cannot
leave two processes fighting over the robot.

## Safety notes for RL

- `rl_conservative` is the default for a reason: random exploration stays inside
  a small, slow, low-contact envelope. Validate the workspace box on your cell
  before training ([safety.md](safety.md)).
- Out-of-box targets are clipped and oversized steps rejected, with the reason
  surfaced in the observation's `stop_reason` bookkeeping — log it to spot a
  policy that keeps slamming a limit.
- Keep an E-stop within reach during real data collection regardless of the
  software guardrails.
