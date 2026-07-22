"""End-to-end validation: every preset vehicle, integrated with the real
6DOF equations of motion, must reach ground impact within sane physical
bounds. These bounds reflect the source sheet's own historical data
(presets.py is copied verbatim, not tuned) run through correct physics -
see the comment in presets.py for provenance. Bounds have headroom for
minor model tuning, not exact values.
"""

import pytest

from sixdof.guidance import GuidanceProgram
from sixdof.presets import PRESETS
from sixdof.simulation import Simulation
from analysis.metrics import compute_metrics

BOUNDS = {
    "Germany - V2":       dict(apogee=(50, 200),    rng=(5, 60),      mach=(1, 10), g=(0, 40)),
    "Russia - Scud-B":    dict(apogee=(0.2, 20),    rng=(2, 40),      mach=(0.3, 6), g=(0, 40)),
    "Iraq - Al-Husayn":   dict(apogee=(5, 100),     rng=(50, 400),    mach=(2, 15), g=(0, 60)),
    "DPRK - Nodong-A":    dict(apogee=(150, 700),   rng=(50, 400),    mach=(3, 20), g=(0, 60)),
    "DPRK - Nodong-A1":   dict(apogee=(200, 900),   rng=(50, 400),    mach=(3, 25), g=(0, 60)),
    "DPRK - Nodong-B":    dict(apogee=(3, 60),      rng=(30, 300),    mach=(1, 12), g=(0, 40)),
    "DPRK - TD-1":        dict(apogee=(100, 700),   rng=(500, 3000),  mach=(5, 30), g=(0, 60)),
    "DPRK - TD-2":        dict(apogee=(700, 3500),  rng=(1000, 5000), mach=(8, 40), g=(0, 60)),
}


def _run(name, trajectory_type="gravity_turn"):
    vehicle = PRESETS[name]
    guidance = GuidanceProgram(launch_azimuth_deg=90, trajectory_type=trajectory_type)
    sim = Simulation(vehicle, guidance, launch_lat_deg=35.0, launch_lon_deg=45.0, t_max=3600)
    return sim.run()


@pytest.mark.parametrize("name", list(PRESETS.keys()))
def test_preset_reaches_impact_within_bounds(name):
    result = _run(name)
    assert result.impacted, f"{name}: did not reach impact within the time limit"

    metrics = compute_metrics(result, PRESETS[name])
    bounds = BOUNDS[name]
    assert bounds["apogee"][0] < metrics.apogee_km < bounds["apogee"][1], \
        f"{name}: apogee out of range: {metrics.apogee_km}"
    assert bounds["rng"][0] < metrics.total_range_km < bounds["rng"][1], \
        f"{name}: downrange out of range: {metrics.total_range_km}"
    assert bounds["mach"][0] < metrics.max_mach < bounds["mach"][1], \
        f"{name}: max Mach out of range: {metrics.max_mach}"
    assert bounds["g"][0] <= metrics.max_accel_g < bounds["g"][1], \
        f"{name}: max axial load out of range: {metrics.max_accel_g}"


def test_flight_is_monotonic_in_time_and_starts_at_zero():
    result = _run("Russia - Scud-B")
    df = result.dataframe
    assert df["time"].iloc[0] == pytest.approx(0.0, abs=1e-6)
    assert df["time"].is_monotonic_increasing


def test_mass_only_decreases_during_boost_and_never_below_payload():
    result = _run("Russia - Scud-B")
    df = result.dataframe
    vehicle = PRESETS["Russia - Scud-B"]
    boost = df[df["powered"]]
    assert boost["mass_kg"].is_monotonic_decreasing
    assert df["mass_kg"].min() >= vehicle.payload_mass - 1e-6


def test_guidance_authority_withdrawn_after_burnout():
    """Once no stage is thrusting, no control torque should be commanded -
    the vehicle must coast/re-enter freely, which is the entire point of
    doing this in 6DOF instead of the old planar model.

    Uses Nodong-A rather than Scud-B: Scud-B's sourced thrust-to-weight is
    only 1.13 and it impacts mid-burn (see README), so it never reaches a
    coast phase at all - not a bug, just not the right fixture for this
    assertion."""
    result = _run("DPRK - Nodong-A")
    df = result.dataframe
    unpowered = df[~df["powered"]]
    assert len(unpowered) > 0
    assert (unpowered["control_torque_Nm"].abs() < 1e-9).all()


def test_fixed_pitch_and_gravity_turn_give_different_ranges():
    """Sanity check that the two guidance modes actually change the
    trajectory rather than silently behaving identically."""
    gt = compute_metrics(_run("Russia - Scud-B", "gravity_turn"))
    fp = compute_metrics(_run("Russia - Scud-B", "fixed_pitch"))
    assert gt.total_range_km != pytest.approx(fp.total_range_km, rel=1e-3)
