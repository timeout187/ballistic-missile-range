import json

import pytest

from analysis.export import dataframe_to_csv_bytes, dataframe_to_json_bytes, metrics_to_json_bytes
from analysis.metrics import compute_metrics, metrics_table
from sixdof.guidance import GuidanceProgram
from sixdof.presets import PRESETS
from sixdof.simulation import Simulation


@pytest.fixture(scope="module")
def scud_result():
    vehicle = PRESETS["Russia - Scud-B"]
    guidance = GuidanceProgram(launch_azimuth_deg=90, trajectory_type="gravity_turn")
    sim = Simulation(vehicle, guidance, launch_lat_deg=35.0, launch_lon_deg=45.0, t_max=3600)
    return sim.run()


def test_compute_metrics_apogee_occurs_before_impact(scud_result):
    m = compute_metrics(scud_result, PRESETS["Russia - Scud-B"])
    assert 0 <= m.apogee_time_s <= m.total_flight_time_s


def test_compute_metrics_max_values_are_at_least_burnout_values(scud_result):
    m = compute_metrics(scud_result, PRESETS["Russia - Scud-B"])
    assert m.max_speed_ms >= m.burnout_speed_ms - 1e-6


def test_compute_metrics_raises_on_empty_dataframe():
    import pandas as pd

    from sixdof.simulation import SimulationResult

    empty = SimulationResult(
        dataframe=pd.DataFrame(), launch_lat=0.0, launch_lon=0.0,
        vehicle_name="test", guidance=GuidanceProgram(),
        impacted=False, termination_reason="n/a",
    )
    with pytest.raises(ValueError):
        compute_metrics(empty)


def test_stability_note_reflects_stable_vehicle(scud_result):
    m = compute_metrics(scud_result, PRESETS["Russia - Scud-B"])
    assert "stable" in m.stability_note.lower()


def test_metrics_table_returns_label_value_pairs(scud_result):
    m = compute_metrics(scud_result, PRESETS["Russia - Scud-B"])
    rows = metrics_table(m)
    assert len(rows) > 5
    for label, value in rows:
        assert isinstance(label, str) and isinstance(value, str)


def test_export_csv_bytes_round_trip(scud_result):
    csv_bytes = dataframe_to_csv_bytes(scud_result.dataframe)
    assert isinstance(csv_bytes, bytes)
    text = csv_bytes.decode("utf-8")
    assert "time" in text.splitlines()[0]


def test_export_json_bytes_are_valid_json(scud_result):
    df_json = dataframe_to_json_bytes(scud_result.dataframe)
    records = json.loads(df_json.decode("utf-8"))
    assert isinstance(records, list) and len(records) > 0

    m = compute_metrics(scud_result, PRESETS["Russia - Scud-B"])
    metrics_json = json.loads(metrics_to_json_bytes(m).decode("utf-8"))
    assert metrics_json["impacted"] is True
