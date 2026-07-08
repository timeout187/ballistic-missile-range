"""Orchestrates a full 6DOF flight: multi-stage boost, staging events,
coast, and ballistic re-entry to impact."""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
import pandas as pd

from .constants import R_EARTH, OMEGA_EARTH
from .dynamics import state_derivative, compute_environment, STATE_SIZE
from .guidance import GuidanceProgram
from .integrator import rk4_step
from .quaternion import quat_between_vectors, euler_from_quat, rotate_body_to_eci
from .vehicle import Vehicle, StageAero


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


@dataclass
class SimulationResult:
    dataframe: pd.DataFrame
    launch_lat: float
    launch_lon: float
    vehicle_name: str
    guidance: GuidanceProgram
    impacted: bool
    termination_reason: str


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

    def run(self, max_steps: int = 400000) -> SimulationResult:
        vehicle = self.vehicle
        guidance = self.guidance
        state = self._initial_state()

        n_stages = len(vehicle.stages)
        stage_idx = 0
        stage_elapsed = 0.0
        mass_before_stage_drop = state[13]

        rows = []
        t = 0.0
        step = 0
        impacted = False
        reason = "reached time limit"

        while step < max_steps:
            r = state[0:3]
            v = state[3:6]
            q = state[6:10]
            omega = state[10:13]
            mass = state[13]
            altitude = float(np.linalg.norm(r) - R_EARTH)

            if step > 5 and altitude <= 0.0:
                impacted = True
                reason = "impact"
                break
            if t > self.t_max:
                reason = "reached time limit"
                break
            if altitude > 5_000_000 or not np.all(np.isfinite(state)):
                reason = "numerical divergence / escape"
                break

            guidance_active = stage_idx < n_stages
            if guidance_active:
                stage = vehicle.stages[stage_idx]
                burning = stage_elapsed < stage.burn_time
                aero = stage.aero
                max_torque = stage.max_control_torque
                inertia = vehicle.inertia_estimate(stage_idx, mass)
            else:
                stage = None
                burning = False
                aero = self.payload_aero
                max_torque = 0.0
                inertia = vehicle.inertia_estimate(n_stages - 1, mass)

            env_now = compute_environment(r, v, q)

            if burning:
                thrust_mag = stage.thrust_at_pressure(env_now.pressure)
                isp_now = stage.isp_at_pressure(env_now.pressure)
                mass_flow = thrust_mag / (isp_now * 9.80665)
            else:
                thrust_mag = 0.0
                mass_flow = 0.0

            # gravity-turn prograde lock must track velocity relative to the
            # rotating Earth/atmosphere, not raw inertial velocity -- the
            # latter already includes the launch site's own ~100s-of-m/s
            # co-rotation speed and would falsely read as "already flying
            # sideways" the instant the vehicle leaves the pad.
            v_rel_for_guidance = v - np.cross(np.array([0.0, 0.0, OMEGA_EARTH]), r)
            active, target_dir = guidance.target_direction(t, v_rel_for_guidance, guidance_active)
            if not active:
                target_dir = None

            def deriv(tt, ss, _thrust=thrust_mag, _mdot=mass_flow, _aero=aero,
                       _inertia=inertia, _target=target_dir, _maxtq=max_torque):
                d, _aux = state_derivative(tt, ss, _mdot, _thrust, _aero,
                                            np.array(_inertia), _target, _maxtq)
                return d

            _, aux = state_derivative(t, state, mass_flow, thrust_mag, aero,
                                       np.array(inertia), target_dir, max_torque)

            lat_deg, lon_deg = eci_to_geodetic(r, t)
            downrange = great_circle_range(self.launch_lat, self.launch_lon, lat_deg, lon_deg)
            roll, pitch, yaw = euler_from_quat(q)
            speed_inertial = float(np.linalg.norm(v))

            x_axis_eci = rotate_body_to_eci(q, np.array([1.0, 0.0, 0.0]))

            rows.append({
                "time": t,
                "stage": stage_idx + 1 if guidance_active else n_stages,
                "powered": burning,
                "altitude_m": altitude,
                "altitude_km": altitude / 1000.0,
                "downrange_m": downrange,
                "downrange_km": downrange / 1000.0,
                "lat_deg": lat_deg,
                "lon_deg": lon_deg,
                "speed_inertial_ms": speed_inertial,
                "speed_relative_ms": aux.env.v_rel_mag,
                "mach": aux.env.mach,
                "mass_kg": mass,
                "thrust_N": thrust_mag,
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
            })

            # use the fine timestep whenever inside the sensibly dense
            # atmosphere (boost AND re-entry both need it to resolve
            # fast aero-restoring oscillations once dynamic pressure is
            # significant); coarsen only for the thin-air exoatmospheric
            # coast, where nothing changes quickly.
            dt = self.dt_boost if (guidance_active or altitude < 40000.0) else self.dt_coast
            state = rk4_step(deriv, t, state, dt)

            t += dt
            step += 1
            if guidance_active:
                stage_elapsed += dt
                if stage_elapsed >= stage.burn_time:
                    state[13] = max(state[13] - stage.dry_mass, self.vehicle.payload_mass)
                    stage_idx += 1
                    stage_elapsed = 0.0

        df = pd.DataFrame(rows)
        if len(df) > 1:
            v_series = df["speed_relative_ms"].to_numpy()
            t_series = df["time"].to_numpy()
            accel = np.gradient(v_series, t_series)
            df["accel_g"] = accel / 9.80665
        elif len(df) == 1:
            df["accel_g"] = 0.0

        return SimulationResult(
            dataframe=df,
            launch_lat=self.launch_lat,
            launch_lon=self.launch_lon,
            vehicle_name=vehicle.name,
            guidance=guidance,
            impacted=impacted,
            termination_reason=reason,
        )
