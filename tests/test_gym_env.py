import numpy as np

from flexiv_control import RobotConfig
from flexiv_control.envs import FlexivRealEnv, make_env


def _env(**kw):
    # Force the fake backend so the test never touches hardware.
    cfg = RobotConfig(backend="fake", control_hz=50.0)
    return FlexivRealEnv(robot=None, config=cfg, control_hz=50.0, **kw)


def test_spaces_shapes():
    env = _env()
    assert env.action_space.shape == (7,)
    assert env.observation_space.shape == (28,)
    a = env.action_space.sample()
    assert a.shape == (7,)
    assert np.all(a >= -1.0 - 1e-6) and np.all(a <= 1.0 + 1e-6)
    env.close()


def test_reset_returns_obs_info():
    env = _env()
    obs, info = env.reset()
    assert obs.shape == (28,)
    assert np.all(np.isfinite(obs))
    assert isinstance(info, dict)
    env.close()


def test_step_signature_and_obs():
    env = _env(max_episode_steps=5)
    env.reset()
    out = env.step(np.zeros(7, dtype=np.float32))
    assert len(out) == 5
    obs, reward, terminated, truncated, info = out
    assert obs.shape == (28,)
    assert isinstance(reward, float)
    assert isinstance(terminated, bool) and isinstance(truncated, bool)
    env.close()


def test_positive_x_action_moves_tcp():
    env = _env()
    env.reset()
    # obs layout: q(7), dq(7), tcp_pose(7) -> tcp x at index 14.
    x0 = env._obs()[14]
    for _ in range(5):
        a = np.zeros(7, dtype=np.float32)
        a[0] = 1.0  # max +x
        obs, *_ = env.step(a)
    assert obs[14] > x0
    env.close()


def test_truncation_at_max_steps():
    env = _env(max_episode_steps=3)
    env.reset()
    truncated = False
    for _ in range(3):
        _, _, terminated, truncated, _ = env.step(np.zeros(7, dtype=np.float32))
    assert truncated is True
    env.close()


def test_make_env_factory():
    env = make_env(robot=None, config=RobotConfig(backend="fake"), control_hz=20.0)
    assert isinstance(env, FlexivRealEnv)
    env.close()
