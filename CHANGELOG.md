# Changelog

All notable changes to this project are documented here. The format is based on
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and this project adheres
to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

Post-0.1.0 hardening from a conformance audit against the industry real-robot
stack (Polymetis / Deoxys / frankapy / SERL / LeRobot / ros2_control + the Flexiv
RDK). All changes are backward compatible; new safety features are opt-in.

### Added
- Per-tick robot-fault gate: `RobotBackend.in_fault()` (RDK `robot.fault()`),
  surfaced as `FAULT`/`BACKEND_FAULT`; the control loop halts streaming on a fault.
- Safety filter: opt-in per-tick acceleration cap (`max_linear_accel`), the
  `STALE_STATE` state-age watchdog (`stop_on_state_stale`), bounded-hold
  escalation (`hold_timeout_ms`), and attributable clip reasons.
- `GripperCommand.from_signed_action` — the unified `[-1, 1]` gripper encoding.
- Mesh-free MuJoCo DLS-IK regression test, run in a new CI `sim` job.
- `py.typed` (PEP 561); single-sourced version; `CHANGELOG.md`; `SECURITY.md`;
  `CODE_OF_CONDUCT.md`; issue/PR templates.

### Changed
- `FlexivRealEnv.reset()` re-enters Cartesian mode after homing (the first
  `step()` would otherwise be rejected on hardware).
- MuJoCo `control_dt` defaults to the control period so chunks play back at the
  right speed in sim; `move_gripper()` now steps the sim.
- RDK backend: gripper `Init()`/`moving()`, `ClearFault()` return checked, DoF
  adopted from the robot, configured `target_wrench` actually commanded.
- Honest labeling: `RemotePolicyClient` speaks flexiv_control's own JSON (not the
  openpi/pi0 wire format); rotation deltas named axis-angle (`drx/dry/drz`);
  "real-time chunking" relabeled "async infer-ahead".
- GitHub Actions bumped to Node-24 runtimes; PyPI publish is idempotent
  (`skip-existing`) and bound to a gated `pypi` environment.

## [0.1.0] - 2026-06-08

First public release.

### Added
- Unified action contract: `CartesianChunk` / `CartesianDelta` / `JointChunk` +
  `GripperCommand` and a quantified `ExecutionResult`.
- First-class safety: named, version-controlled `SafetyProfile`s and a
  microsecond per-tick `SafetyFilter` (workspace box, speed/jump caps, joint
  limits, contact-wrench stop, command watchdog).
- Backends behind one config switch: real Rizon (Flexiv RDK v1.x), a
  dependency-free fake, and MuJoCo (Jacobian DLS IK, gravity compensation, GN01
  gripper).
- Cross-process server + `RemoteRobot` with an in-process lease and a host-wide
  lock; single-writer, hold-on-stale `ReactiveServoLoop`.
- Optional C++ 1 kHz RT daemon and a ROS 2 overlay.
- Gymnasium env + HIL-SERL intervention wrapper; LeRobot adapter; SpaceMouse
  teleop; `RecedingHorizonRunner` + `RemotePolicyClient` (the policy-server seam).
- PyPI Trusted Publishing release workflow.

[Unreleased]: https://github.com/ZihaoLu001/flexiv_control/compare/v0.1.0...HEAD
[0.1.0]: https://github.com/ZihaoLu001/flexiv_control/releases/tag/v0.1.0
