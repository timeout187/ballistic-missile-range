import math

import numpy as np
import pytest

from sixdof.guidance import GuidanceProgram, minimum_energy_gamma


def _launched_guidance(**kwargs) -> GuidanceProgram:
    g = GuidanceProgram(**kwargs)
    g.set_launch_frame(np.array([6371000.0, 0.0, 0.0]))
    return g


def test_guidance_withdrawn_when_not_powered():
    g = _launched_guidance()
    active, direction = g.target_direction(t=50.0, vel_eci=np.array([100.0, 0, 0]), powered=False)
    assert active is False
    assert direction is None


def test_vertical_rise_points_straight_up():
    g = _launched_guidance(t_vertical=6.0)
    active, direction = g.target_direction(t=2.0, vel_eci=np.zeros(3), powered=True)
    assert active is True
    assert np.allclose(direction, g._up0)


def test_pitch_kick_tilts_away_from_vertical_over_time():
    g = _launched_guidance(t_vertical=6.0, t_pitch=10.0, pitch_kick_deg=5.0)
    _, dir_start = g.target_direction(t=6.0, vel_eci=np.zeros(3), powered=True)
    _, dir_mid = g.target_direction(t=11.0, vel_eci=np.zeros(3), powered=True)
    angle_start = math.degrees(math.acos(np.clip(np.dot(dir_start, g._up0), -1, 1)))
    angle_mid = math.degrees(math.acos(np.clip(np.dot(dir_mid, g._up0), -1, 1)))
    assert angle_start == pytest.approx(0.0, abs=1e-6)
    assert angle_mid > angle_start
    assert angle_mid < 5.0 + 1e-6


def test_gravity_turn_prograde_locks_onto_velocity():
    g = _launched_guidance(t_vertical=1.0, t_pitch=1.0, trajectory_type="gravity_turn")
    vel = np.array([300.0, 0.0, 100.0])
    active, direction = g.target_direction(t=10.0, vel_eci=vel, powered=True)
    assert active is True
    assert np.allclose(direction, vel / np.linalg.norm(vel))


def test_fixed_pitch_holds_constant_inertial_angle_regardless_of_velocity():
    g = _launched_guidance(t_vertical=1.0, t_pitch=1.0, trajectory_type="fixed_pitch",
                            boost_pitch_deg=30.0)
    _, dir_a = g.target_direction(t=10.0, vel_eci=np.array([100.0, 0, 0]), powered=True)
    _, dir_b = g.target_direction(t=50.0, vel_eci=np.array([500.0, 0, 300]), powered=True)
    assert np.allclose(dir_a, dir_b, atol=1e-9)
    angle_from_horizon = 90.0 - math.degrees(math.acos(np.clip(np.dot(dir_a, g._up0), -1, 1)))
    assert angle_from_horizon == pytest.approx(30.0, abs=1e-6)


def test_minimum_energy_gamma_is_45deg_for_zero_range():
    assert math.degrees(minimum_energy_gamma(0.0)) == pytest.approx(45.0, abs=1e-6)


def test_minimum_energy_gamma_matches_wright_1992_formula():
    """gamma_burnout = 1/2 * atan(sin(phi) / (cos(phi) - 1)), per the Wright
    (1992) formula this function implements (see its docstring) -- checked
    directly against the closed-form rather than an assumed monotonicity,
    since that formula is not "gamma decreases with range" in this
    parametrization (it works out to 45 deg + phi/4)."""
    for phi_deg in (10.0, 45.0, 90.0, 150.0):
        phi = math.radians(phi_deg)
        expected = 0.5 * math.atan2(math.sin(phi), math.cos(phi) - 1.0)
        assert minimum_energy_gamma(phi) == pytest.approx(expected, abs=1e-9)
