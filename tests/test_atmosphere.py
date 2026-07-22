import math

import pytest

from sixdof.atmosphere import atmosphere
from sixdof.constants import GAMMA_AIR, R_AIR


def test_sea_level_matches_iso_standard():
    rho, p, t, a = atmosphere(0.0)
    assert p == pytest.approx(101325.0, rel=1e-6)
    assert t == pytest.approx(288.15, rel=1e-6)
    assert rho == pytest.approx(1.225, rel=2e-3)
    assert a == pytest.approx(340.3, rel=2e-3)


def test_negative_altitude_clamped_to_sea_level():
    assert atmosphere(-500.0) == atmosphere(0.0)


@pytest.mark.parametrize("h", [500.0, 5000.0, 10999.0, 15000.0, 30000.0, 50000.0, 80000.0])
def test_density_pressure_temperature_positive_and_decreasing(h):
    rho, p, t, a = atmosphere(h)
    rho_below, p_below, _, _ = atmosphere(0.0)
    assert rho > 0 and p > 0 and t > 0 and a > 0
    assert rho < rho_below
    assert p < p_below


def test_density_monotonically_decreases_with_altitude():
    altitudes = [0, 5000, 11000, 20000, 32000, 47000, 51000, 71000, 86000, 100000, 150000]
    densities = [atmosphere(h)[0] for h in altitudes]
    for lo, hi in zip(densities, densities[1:]):
        assert hi < lo


def test_ideal_gas_law_is_self_consistent():
    for h in (0.0, 8000.0, 20000.0, 45000.0, 70000.0):
        rho, p, t, _ = atmosphere(h)
        assert p == pytest.approx(rho * R_AIR * t, rel=1e-6)


def test_speed_of_sound_matches_temperature():
    for h in (0.0, 11000.0, 20000.0, 50000.0):
        rho, p, t, a = atmosphere(h)
        assert a == pytest.approx(math.sqrt(GAMMA_AIR * R_AIR * t), rel=1e-9)


def test_above_86km_tail_stays_positive_and_decays():
    rho_86 = atmosphere(86000.0)[0]
    rho_120 = atmosphere(120000.0)[0]
    rho_200 = atmosphere(200000.0)[0]
    assert rho_86 > rho_120 > rho_200 > 0
