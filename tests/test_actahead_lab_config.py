"""The ActAhead lab config + safety profile load and carry the real cell limits."""

from __future__ import annotations

from flexiv_control import Robot, RobotConfig, load_safety_profile


def test_actahead_lab_robot_config_loads():
    c = RobotConfig.load("rizon4s_actahead_lab")
    assert c.robot_id == "rizon4s_actahead_lab"
    assert c.robot_sn == "Rizon4s-062626"
    assert c.default_safety_profile == "actahead_lab"
    assert c.q_home.shape == (7,)
    assert abs(float(c.q_home[3]) - 1.573) < 1e-3  # joint4 from the lab home pose


def test_actahead_lab_safety_profile_loads():
    p = load_safety_profile("actahead_lab")
    assert p.name == "actahead_lab"
    assert tuple(p.ws_x) == (0.25, 0.85)
    assert tuple(p.ws_y) == (-0.35, 0.35)
    assert abs(p.max_linear_speed - 0.05) < 1e-9
    assert abs(p.max_angular_speed - 0.20) < 1e-9
    assert abs(p.command_timeout_ms - 200.0) < 1e-9


def test_robot_uses_actahead_profile_end_to_end():
    r = Robot(RobotConfig.load("rizon4s_actahead_lab"))
    assert r.profile.name == "actahead_lab"
    assert r.profile.ws_x[1] == 0.85
    r.connect()  # fake backend; just confirm it constructs + connects
    assert r.get_state().q.shape == (7,)
    r.disconnect()
