# Contributing to flexiv_control

Thanks for your interest in improving `flexiv_control`. This is a community
project (not affiliated with Flexiv Robotics) and contributions of all sizes —
bug reports, docs, new backends, integration adapters — are welcome.

## Ground rules

- **Safety first.** This software commands real robot arms. Any change to the
  safety supervisor, interpolation, control loop, or a backend must keep the
  hardware-free tests green and must not weaken a default safety profile without
  a clear, documented reason. When in doubt, make the safe behaviour the default
  and the relaxed behaviour opt-in.
- **The core stays numpy-only.** New third-party dependencies belong in an
  optional extra in `pyproject.toml` (`[project.optional-dependencies]`), guarded
  by a lazy import, never in the core import path.
- **The action contract is the spine.** New consumers (planners, envs, teleop)
  should emit the existing `CartesianChunk` / `CartesianDelta` / `JointChunk`
  rather than inventing a new robot-facing API. New backends should consume the
  same filtered setpoint stream. See [docs/action_contract.md](docs/action_contract.md).

## Development setup

```bash
git clone https://github.com/ZihaoLu001/flexiv_control.git
cd flexiv_control
pip install -e ".[dev]"      # pytest + ruff; add rl/teleop/lerobot/mujoco as needed
```

Lint and test:

```bash
ruff check src tests examples
pytest -q
```

CI (`.github/workflows/ci.yml`) runs the suite on Python 3.8 / 3.10 / 3.12 and,
separately, verifies that the numpy-only core installs and runs a fake-backend
smoke test. Please make sure both would pass before opening a PR.

## Tests

- All tests must run on the **`fake` backend** with no hardware and no heavy
  optional dependency. Hardware/sim-specific logic is exercised through the
  backend seam, not by requiring a real robot in CI.
- Add a test with any bug fix or new feature. Put it under `tests/` and keep it
  fast and deterministic.

## Working with hardware-specific code

The real-hardware (`flexiv_rdk`), MuJoCo, LeRobot, and C++/ROS paths contain
`# VERIFY:` comments wherever they call an external API whose name/signature can
differ by version. If you touch these:

- Keep (and update) the `# VERIFY:` markers so the next person knows what to
  confirm against their install. See [docs/versions.md](docs/versions.md).
- Do not silently pin an assumption you could not test — note it.

## Style

- Python formatted/linted with `ruff` (line length 100, target py38). Run
  `ruff check` (and fix) before committing.
- Prefer plain dataclasses + numpy and clear docstrings that say *why*, not just
  *what*. Match the surrounding style.
- Keep public API changes documented: update the relevant file under `docs/` and
  the README table in the same PR.

## Submitting a change

1. Open an issue describing the bug/feature first for anything non-trivial.
2. Branch, make focused commits, and keep the PR scoped to one thing.
3. Ensure `ruff check` and `pytest -q` pass locally.
4. Describe the change, the motivation, and (for behaviour changes) what you
   tested — including whether you validated on real hardware and at what speed.

## License of contributions

By contributing, you agree that your contributions are licensed under the
project's [Apache-2.0](LICENSE) license, per Section 5 of that license.
