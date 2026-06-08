## Summary

What this PR changes and why.

## Changes

-

## Testing

- [ ] `pytest -q` passes
- [ ] `ruff check src tests examples` clean
- [ ] If it touches the MuJoCo backend: `pytest tests/test_mujoco_ik_meshless.py`
- [ ] If it touches the real-hardware (`flexiv_rdk`) path: reviewed the
      `# VERIFY:` markers; noted that it is hardware-unvalidated

## Notes for reviewers

Anything non-obvious — design trade-offs, follow-ups, or safety implications.
