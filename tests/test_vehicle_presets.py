import math

import pytest

from sixdof.presets import PRESETS
from sixdof.vehicle import Stage, StageAero, Vehicle


def test_all_presets_are_flyable_at_liftoff():
    """Every preset's first stage must produce more thrust than the
    vehicle's liftoff weight, or it could never leave the pad."""
    for name, vehicle in PRESETS.items():
        stage0 = vehicle.stages[0]
        weight = vehicle.total_mass() * 9.81
        assert stage0.thrust_vac > weight, f"{name}: T/W <= 1 at liftoff"


def test_presets_have_at_least_one_stage_and_positive_masses():
    for name, vehicle in PRESETS.items():
        assert len(vehicle.stages) >= 1, name
        for stage in vehicle.stages:
            assert stage.dry_mass > 0
            assert stage.propellant_mass > 0
            assert stage.isp_vac > 0
            assert stage.thrust_vac > 0
            assert stage.diameter > 0


def test_stage_burn_time_is_autocomputed_from_propellant_and_mdot():
    stage = Stage(name="test", dry_mass=100, propellant_mass=500,
                  isp_sea=200, isp_vac=220, thrust_vac=10000, diameter=0.5)
    expected_mdot = stage.thrust_vac / (stage.isp_vac * 9.80665)
    assert stage.mass_flow_rate() == pytest.approx(expected_mdot)
    assert stage.burn_time == pytest.approx(stage.propellant_mass / expected_mdot)


def test_stage_thrust_and_isp_interpolate_between_sea_level_and_vacuum():
    stage = Stage(name="test", dry_mass=100, propellant_mass=500,
                  isp_sea=200, isp_vac=250, thrust_vac=10000, diameter=0.5)
    assert stage.thrust_at_pressure(0.0) == pytest.approx(10000.0)
    assert stage.isp_at_pressure(0.0) == pytest.approx(250.0)
    sea_level_thrust = stage.thrust_at_pressure(101325.0)
    expected_sea_thrust = 10000.0 * (200.0 / 250.0)
    assert sea_level_thrust == pytest.approx(expected_sea_thrust)
    assert stage.isp_at_pressure(101325.0) == pytest.approx(200.0)


def test_vehicle_total_mass_sums_stages_and_payload():
    vehicle = Vehicle(
        name="test",
        payload_mass=100,
        stages=[
            Stage(name="s1", dry_mass=50, propellant_mass=200, isp_sea=200,
                  isp_vac=220, thrust_vac=5000, diameter=0.5),
            Stage(name="s2", dry_mass=20, propellant_mass=80, isp_sea=250,
                  isp_vac=270, thrust_vac=2000, diameter=0.4),
        ],
    )
    assert vehicle.total_mass() == pytest.approx(100 + 50 + 200 + 20 + 80)


def test_inertia_estimate_scales_with_mass_and_is_positive():
    vehicle = PRESETS["Russia - Scud-B"]
    ixx_light, iyy_light = vehicle.inertia_estimate(0, current_mass=1000.0)
    ixx_heavy, iyy_heavy = vehicle.inertia_estimate(0, current_mass=5000.0)
    assert ixx_light > 0 and iyy_light > 0
    assert ixx_heavy > ixx_light
    assert iyy_heavy > iyy_light


def test_stage_aero_cd0_peaks_near_transonic():
    aero = StageAero(ref_area=1.0, ref_length=1.0)
    cd_sub = aero.cd0(0.5)
    cd_trans = aero.cd0(1.0)
    cd_super = aero.cd0(3.0)
    assert cd_trans > cd_sub
    assert cd_trans > cd_super


def test_stage_aero_cd0_is_continuous_across_regime_boundaries():
    aero = StageAero(ref_area=1.0, ref_length=1.0)
    eps = 1e-4
    assert aero.cd0(0.8 - eps) == pytest.approx(aero.cd0(0.8 + eps), abs=1e-2)
    assert aero.cd0(1.2 - eps) == pytest.approx(aero.cd0(1.2 + eps), abs=1e-2)
