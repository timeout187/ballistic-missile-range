"""Orchestrates a full 6DOF flight: multi-stage boost, staging events,
coast, and ballistic re-entry to impact.

Integration uses scipy's adaptive-step solve_ivp (RK45) rather than a
fixed-step integrator. This matters for correctness, not just convenience:
during fast, dense-atmosphere re-entry the aerodynamic force and moment
are strongly nonlinear in velocity and dynamic pressure, which makes the
equations locally stiff - a fixed timestep that is fine during boost can
fall outside an explicit fixed-step method's stability region during
re-entry and diverge exponentially even though the underlying physics is
just a damped oscillation. An adaptive solver shrinks its internal step
automatically wherever the local error estimate demands it, which is the
robust general solution rather than hand-tuned step-size heuristics.

The flight is split into physically distinct, individually-continuous
phases (each stage's burn, an exoatmospheric coast segment, and a dense-
atmosphere re-entry segment), each integrated with its own solve_ivp call
and its own termination event (ground impact, or an altitude threshold
crossing). Report rows are sampled from each phase's dense output at a
fixed cadence chosen for that phase - this sampling is decoupled from the
solver's own adaptive step choices (dense-output interpolation is cheap),
so output resolution stays fine without forcing the solver itself to take
tiny steps where it doesn't need to.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
import pandas as pd
from scipy.integrate import solve_ivp

from .constants import R_EARTH, OMEGA_EARTH, G0
from .dynamics import state_derivative, compute_environment, STATE_SIZE
from .guidance import GuidanceProgram
from .quaternion import quat_between_vectors, quat_normalize, euler_from_quat, rotate_body_to_eci
from .vehicle import Vehicle, StageAero

RE_ENTRY_DENSE_ALT_M = 40000.0  # below this, sample/track finely on the way down
IMPACT_ALT_TOL_M = 0.0


def _payload_aero(vehicle: Vehicle) -> StageAero:
    d = vehicle.payload_diameter
    area = math.pi * (d / 2.0) ** 2
    return StageAero(
        ref_area=area, ref_length=d,
        cd0_subsonic=vehicle.payload_cd0,
        cd0_transonic=vehicle.payload_cd0 * 1.8,
        cd0_supersonic=vehicle.payload_cd0 * 0.9,
        k_alpha=0.3,
        cl_alpha=vehicle.payload_cl_alpha,
        cm0=0.0,
        cm_alpha=vehicle.payload_cm_alpha,
        cm_q=4.0,
    )


def eci_to_geodetic(r_eci: np.ndarray, t: float):
    theta = OMEGA_EARTH * t
    ct, st = math.cos(theta), math.sin(theta)
    x, y, z = r_eci
    x_ecef = ct * x + st * y
    y_ecef = -st * x + ct * y
    z_ecef = z
    r_norm = math.sqrt(x_ecef ** 2 + y_ecef ** 2 + z_ecef ** 2)
    lat = math.degrees(math.asin(max(-1.0, min(1.0, z_ecef / r_norm))))
    lon = math.degrees(math.atan2(y_ecef, x_ecef))
    return lat, lon


def great_circle_range(lat1_deg, lon1_deg, lat2_deg, lon2_deg) -> float:
    lat1, lon1, lat2, lon2 = map(math.radians, (lat1_deg, lon1_deg, lat2_deg, lon2_deg))
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = math.sin(dlat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    return R_EARTH * c


def _altitude(y: np.ndarray) -> float:
    return float(np.linalg.norm(y[0:3]) - R_EARTH)


@dataclass
class SimulationResult:
    dataframe: pd.DataFrame
    launch_lat: float
    launch_lon: float
    vehicle_name: str
    guidance: GuidanceProgram
    impacted: bool
    termination_reason: str


class _Phase:
    """A physically-continuous segment of flight: fixed aero model, fixed
    guidance-authority flag, and either a fixed thrusting stage or none."""

    def __init__(self, aero, inertia_stage_idx, max_torque, guidance_active,
                 display_stage, stage=None):
        self.aero = aero
        self.inertia_stage_idx = inertia_stage_idx
        self.max_torque = max_torque
        self.guidance_active = guidance_active
        self.display_stage = display_stage
        self.stage = stage  # Stage if thrusting this phase, else None


class Simulation:
    def __init__(self, vehicle: Vehicle, guidance: GuidanceProgram,
                 launch_lat_deg: float = 0.0, launch_lon_deg: float = 0.0,
                 dt_boost: float = 0.02, dt_coast: float = 0.25, t_max: float = 3600.0):
        self.vehicle = vehicle
        self.guidance = guidance
        self.launch_lat = launch_lat_deg
        self.launch_lon = launch_lon_deg
        self.dt_boost = dt_boost
        self.dt_coast = dt_coast
        self.t_max = t_max
        self.payload_aero = _payload_aero(vehicle)

    def _initial_state(self) -> np.ndarray:
        lat = math.radians(self.launch_lat)
        lon = math.radians(self.launch_lon)
        r0 = np.array([
            R_EARTH * math.cos(lat) * math.cos(lon),
            R_EARTH * math.cos(lat) * math.sin(lon),
            R_EARTH * math.sin(lat),
        ])
        omega_vec = np.array([0.0, 0.0, OMEGA_EARTH])
        v0 = np.cross(omega_vec, r0)  # co-rotating with the launch site

        up0 = r0 / np.linalg.norm(r0)
        q0 = quat_between_vectors(np.array([1.0, 0.0, 0.0]), up0)

        state = np.zeros(STATE_SIZE)
        state[0:3] = r0
        state[3:6] = v0
        state[6:10] = q0
        state[10:13] = 0.0
        state[13] = self.vehicle.total_mass()

        self.guidance.set_launch_frame(r0)
        return state

    def _rhs(self, t: float, y: np.ndarray, phase: "_Phase") -> np.ndarray:
        dstate, _aux = self._rhs_with_aux(t, y, phase)
        return dstate

    def _row_at(self, t: float, y: np.ndarray, phase: "_Phase") -> dict:
        r, v, qq, omega, mass = y[0:3], y[3:6], y[6:10], y[10:13], max(y[13], 1e-3)
        qq = quat_normalize(qq)

        _dstate, aux = self._rhs_with_aux(t, y, phase)

        altitude = _altitude(y)
        lat_deg, lon_deg = eci_to_geodetic(r, t)
        downrange = great_circle_range(self.launch_lat, self.launch_lon, lat_deg, lon_deg)
        roll, pitch, yaw = euler_from_quat(qq)
        x_axis_eci = rotate_body_to_eci(qq, np.array([1.0, 0.0, 0.0]))

        return {
            "time": t,
            "stage": phase.display_stage,
            "powered": phase.stage is not None,
            "altitude_m": altitude,
            "altitude_km": altitude / 1000.0,
            "downrange_m": downrange,
            "downrange_km": downrange / 1000.0,
            "lat_deg": lat_deg,
            "lon_deg": lon_deg,
            "speed_inertial_ms": float(np.linalg.norm(v)),
            "speed_relative_ms": aux.env.v_rel_mag,
            "mach": aux.env.mach,
            "mass_kg": mass,
            "thrust_N": aux.thrust_mag,
            "drag_N": aux.drag_mag,
            "dynamic_pressure_Pa": aux.env.q_dynamic,
            "alpha_total_deg": math.degrees(aux.env.alpha_total),
            "roll_deg": math.degrees(roll),
            "pitch_deg": math.degrees(pitch),
            "yaw_deg": math.degrees(yaw),
            "omega_x_dps": math.degrees(omega[0]),
            "omega_y_dps": math.degrees(omega[1]),
            "omega_z_dps": math.degrees(omega[2]),
            "control_torque_Nm": float(np.linalg.norm(aux.control_torque)),
            "nose_x_eci": x_axis_eci[0],
            "nose_y_eci": x_axis_eci[1],
            "nose_z_eci": x_axis_eci[2],
            "x_eci": r[0], "y_eci": r[1], "z_eci": r[2],
            "vx_eci": v[0], "vy_eci": v[1], "vz_eci": v[2],
        }

    def _rhs_with_aux(self, t: float, y: np.ndarray, phase: "_Phase"):
        r, v, qq, mass = y[0:3], y[3:6], y[6:10], max(y[13], 1e-3)
        env = compute_environment(r, v, qq)

        if phase.stage is not None:
            thrust_mag = phase.stage.thrust_at_pressure(env.pressure)
            isp_now = phase.stage.isp_at_pressure(env.pressure)
            mass_flow = thrust_mag / (isp_now * G0)
        else:
            thrust_mag, mass_flow = 0.0, 0.0

        if phase.guidance_active:
            v_rel_for_guidance = v - np.cross(np.array([0.0, 0.0, OMEGA_EARTH]), r)
            active, target_dir = self.guidance.target_direction(t, v_rel_for_guidance, True)
            if not active:
                target_dir = None
        else:
            target_dir = None

        inertia = self.vehicle.inertia_estimate(phase.inertia_stage_idx, mass)
        return state_derivative(t, y, mass_flow, thrust_mag, phase.aero,
                                 np.array(inertia), target_dir, phase.max_torque)

    def _integrate_phase(self, t0: float, y0: np.ndarray, t_end: float, phase: "_Phase",
                          max_step: float, sample_dt: float, extra_event=None):
        """Integrate one continuous phase with solve_ivp; always terminates on
        ground impact. Returns (sol_or_None, t_stop, y_stop, rows, hit_ground,
        hit_extra_event)."""

        def rhs(t, y):
            return self._rhs(t, y, phase)

        def impact_event(t, y):
            return _altitude(y)
        impact_event.terminal = True
        impact_event.direction = -1

        events = [impact_event]
        if extra_event is not None:
            events.append(extra_event)

        sol = solve_ivp(
            rhs, (t0, t_end), y0, method="RK45",
            max_step=max_step, rtol=1e-8, atol=1e-6,
            dense_output=True, events=events,
        )

        hit_ground = len(sol.t_events[0]) > 0
        hit_extra = extra_event is not None and len(sol.t_events[1]) > 0

        t_stop = sol.t[-1]
        y_stop = sol.y[:, -1].copy()
        y_stop[6:10] = quat_normalize(y_stop[6:10])

        n_samples = max(2, int(math.ceil((t_stop - t0) / sample_dt)) + 1)
        t_samples = np.linspace(t0, t_stop, n_samples)
        rows = [self._row_at(float(tt), sol.sol(tt), phase) for tt in t_samples]
        # always include the exact stopping point (event crossing) so impact /
        # phase-boundary altitude is captured precisely, not just near it
        if t_samples[-1] != t_stop:
            rows.append(self._row_at(float(t_stop), y_stop, phase))

        return t_stop, y_stop, rows, hit_ground, hit_extra

    def run(self, max_steps: int = 400000) -> SimulationResult:
        vehicle = self.vehicle
        n_stages = len(vehicle.stages)

        state = self._initial_state()
        t = 0.0
        rows: list[dict] = []
        impacted = False
        reason = "reached time limit"

        # --- boost phases: one continuous solve_ivp call per stage ---
        for stage_idx in range(n_stages):
            if t >= self.t_max:
                reason = "reached time limit"
                break
            stage = vehicle.stages[stage_idx]
            phase = _Phase(
                aero=stage.aero, inertia_stage_idx=stage_idx,
                max_torque=stage.max_control_torque, guidance_active=True,
                display_stage=stage_idx + 1, stage=stage,
            )
            t_end = min(t + stage.burn_time, self.t_max)
            t_stop, y_stop, seg_rows, hit_ground, _ = self._integrate_phase(
                t, state, t_end, phase,
                max_step=self.dt_boost, sample_dt=self.dt_boost,
            )
            rows.extend(seg_rows)
            t, state = t_stop, y_stop

            if hit_ground:
                impacted = True
                reason = "impact"
                break
            if t >= self.t_max:
                reason = "reached time limit"
                break

            # instantaneous staging: drop this stage's dry mass
            state[13] = max(state[13] - stage.dry_mass, vehicle.payload_mass)

        # --- coast + re-entry, only if boost finished without impact/timeout ---
        if not impacted and t < self.t_max:
            coast_phase = _Phase(
                aero=self.payload_aero, inertia_stage_idx=n_stages - 1,
                max_torque=0.0, guidance_active=False,
                display_stage=n_stages, stage=None,
            )

            def descending_through_dense_alt(tt, y):
                return _altitude(y) - RE_ENTRY_DENSE_ALT_M
            descending_through_dense_alt.terminal = True
            descending_through_dense_alt.direction = -1

            # Segment A: coarse sampling while above the dense-atmosphere
            # threshold (cheap - can be a long, slowly-changing coast) or
            # until impact if the flight never gets that high.
            t_stop, y_stop, seg_rows, hit_ground, hit_dense = self._integrate_phase(
                t, state, self.t_max, coast_phase,
                max_step=self.dt_coast, sample_dt=self.dt_coast,
                extra_event=descending_through_dense_alt,
            )
            rows.extend(seg_rows)
            t, state = t_stop, y_stop

            if hit_ground:
                impacted = True
                reason = "impact"
            elif t >= self.t_max:
                reason = "reached time limit"
            elif hit_dense:
                # Segment B: fine sampling for the dense-atmosphere re-entry;
                # the adaptive solver also shrinks its own internal step
                # automatically wherever the stiff aero terms demand it, so
                # this segment is numerically stable regardless of how fast
                # things change right before impact.
                t_stop, y_stop, seg_rows, hit_ground, _ = self._integrate_phase(
                    t, state, self.t_max, coast_phase,
                    max_step=self.dt_boost, sample_dt=self.dt_boost,
                )
                rows.extend(seg_rows)
                t, state = t_stop, y_stop
                if hit_ground:
                    impacted = True
                    reason = "impact"
                elif t >= self.t_max:
                    reason = "reached time limit"

        df = pd.DataFrame(rows).drop_duplicates(subset="time").sort_values("time").reset_index(drop=True)
        if len(df) > 1:
            v_series = df["speed_relative_ms"].to_numpy()
            t_series = df["time"].to_numpy()
            accel = np.gradient(v_series, t_series)
            df["accel_g"] = accel / G0
        elif len(df) == 1:
            df["accel_g"] = 0.0

        return SimulationResult(
            dataframe=df,
            launch_lat=self.launch_lat,
            launch_lon=self.launch_lon,
            vehicle_name=vehicle.name,
            guidance=self.guidance,
            impacted=impacted,
            termination_reason=reason,
        )
