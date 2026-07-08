"""Flight-performance metrics computed from a SimulationResult's dataframe."""

from __future__ import annotations

import math
from dataclasses import dataclass

import pandas as pd

from sixdof.simulation import SimulationResult


@dataclass
class FlightMetrics:
    apogee_km: float
    apogee_time_s: float
    total_flight_time_s: float
    burnout_time_s: float
    burnout_altitude_km: float
    burnout_speed_ms: float
    max_speed_ms: float
    max_mach: float
    max_mach_time_s: float
    max_dynamic_pressure_kpa: float
    max_q_time_s: float
    max_accel_g: float
    max_accel_time_s: float
    max_alpha_deg: float
    total_range_km: float
    impact_lat: float
    impact_lon: float
    impact_speed_ms: float
    impact_downrange_km: float
    stability_note: str
    impacted: bool
    termination_reason: str


def _row_at(df: pd.DataFrame, idx) -> pd.Series:
    return df.loc[idx]


def compute_metrics(result: SimulationResult, vehicle=None) -> FlightMetrics:
    df = result.dataframe
    if df.empty:
        raise ValueError("Simulation produced no data to analyze.")

    apogee_idx = df["altitude_km"].idxmax()
    burnout_rows = df[df["powered"]]
    if len(burnout_rows) > 0:
        burnout_row = burnout_rows.iloc[-1]
        burnout_time = float(burnout_row["time"])
        burnout_alt = float(burnout_row["altitude_km"])
        burnout_speed = float(burnout_row["speed_relative_ms"])
    else:
        burnout_time = 0.0
        burnout_alt = 0.0
        burnout_speed = 0.0

    max_mach_idx = df["mach"].idxmax()
    max_q_idx = df["dynamic_pressure_Pa"].idxmax()
    max_g_idx = df["accel_g"].idxmax() if "accel_g" in df else df.index[0]

    last = df.iloc[-1]

    stability_note = "not evaluated"
    if vehicle is not None and len(vehicle.stages) > 0:
        cm_alphas = [s.aero.cm_alpha for s in vehicle.stages]
        payload_cm = vehicle.payload_cm_alpha
        if payload_cm < 0 and all(c < 0 for c in cm_alphas):
            stability_note = (
                "Statically stable in all phases (Cm_alpha < 0 for every stage and the "
                "post-burnout body) - the vehicle weathercocks back toward the relative "
                "wind if disturbed."
            )
        elif payload_cm >= 0:
            stability_note = (
                "The post-burnout body (payload/RV) is statically NEUTRAL or UNSTABLE "
                "(Cm_alpha >= 0) - expect it to tumble once guidance authority is "
                "withdrawn at final burnout, which the angle-of-attack trace should show."
            )
        else:
            stability_note = "Mixed stability across stages - see per-stage Cm_alpha in the vehicle definition."

    return FlightMetrics(
        apogee_km=float(df.loc[apogee_idx, "altitude_km"]),
        apogee_time_s=float(df.loc[apogee_idx, "time"]),
        total_flight_time_s=float(last["time"]),
        burnout_time_s=burnout_time,
        burnout_altitude_km=burnout_alt,
        burnout_speed_ms=burnout_speed,
        max_speed_ms=float(df["speed_relative_ms"].max()),
        max_mach=float(df.loc[max_mach_idx, "mach"]),
        max_mach_time_s=float(df.loc[max_mach_idx, "time"]),
        max_dynamic_pressure_kpa=float(df.loc[max_q_idx, "dynamic_pressure_Pa"]) / 1000.0,
        max_q_time_s=float(df.loc[max_q_idx, "time"]),
        max_accel_g=float(df.loc[max_g_idx, "accel_g"]) if "accel_g" in df else 0.0,
        max_accel_time_s=float(df.loc[max_g_idx, "time"]) if "accel_g" in df else 0.0,
        max_alpha_deg=float(df["alpha_total_deg"].max()),
        total_range_km=float(last["downrange_km"]),
        impact_lat=float(last["lat_deg"]),
        impact_lon=float(last["lon_deg"]),
        impact_speed_ms=float(last["speed_relative_ms"]),
        impact_downrange_km=float(last["downrange_km"]),
        stability_note=stability_note,
        impacted=result.impacted,
        termination_reason=result.termination_reason,
    )


def metrics_table(m: FlightMetrics) -> "list[tuple[str, str]]":
    """Human-readable (label, value) rows for display in the GUI."""
    def fmt_time(s):
        m_, s_ = divmod(s, 60)
        return f"{int(m_)} min {s_:0.1f} s ({s:0.1f} s)"

    return [
        ("Outcome", "Impact" if m.impacted else f"Did not impact ({m.termination_reason})"),
        ("Total flight time", fmt_time(m.total_flight_time_s)),
        ("Burnout time", fmt_time(m.burnout_time_s)),
        ("Burnout altitude", f"{m.burnout_altitude_km:0.2f} km"),
        ("Burnout speed (relative)", f"{m.burnout_speed_ms:0.1f} m/s"),
        ("Apogee", f"{m.apogee_km:0.2f} km at t = {m.apogee_time_s:0.1f} s"),
        ("Max speed (relative to air)", f"{m.max_speed_ms:0.1f} m/s"),
        ("Max Mach", f"{m.max_mach:0.2f} at t = {m.max_mach_time_s:0.1f} s"),
        ("Max dynamic pressure (max-Q)", f"{m.max_dynamic_pressure_kpa:0.1f} kPa at t = {m.max_q_time_s:0.1f} s"),
        ("Max axial load", f"{m.max_accel_g:0.1f} g at t = {m.max_accel_time_s:0.1f} s"),
        ("Max total angle of attack", f"{m.max_alpha_deg:0.1f} deg"),
        ("Total downrange distance", f"{m.total_range_km:0.1f} km"),
        ("Impact point", f"{m.impact_lat:0.3f} deg, {m.impact_lon:0.3f} deg"),
        ("Impact speed (relative)", f"{m.impact_speed_ms:0.1f} m/s"),
        ("Static stability", m.stability_note),
    ]
