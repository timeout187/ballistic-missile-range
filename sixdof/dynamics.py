"""
Six-degree-of-freedom rigid-body equations of motion.

State vector (14 elements), integrated in the Earth-Centered Inertial (ECI)
frame:

    state[0:3]   position r_eci            (m)
    state[3:6]   velocity v_eci            (m/s)
    state[6:10]  attitude quaternion q     (body -> ECI, scalar-first)
    state[10:13] body angular rate omega_b (rad/s)
    state[13]    vehicle mass              (kg)

Gravity is a point-mass central force, so the ECI frame is truly inertial:
no centrifugal/Coriolis pseudo-forces are needed for translation. Coriolis
effects on the *aerodynamics* still matter, because the atmosphere co-rotates
with the Earth -- so aerodynamic forces are computed from the velocity
relative to the local co-rotating air mass, v_rel = v_eci - omega_earth x r.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Optional

import numpy as np

from .atmosphere import atmosphere
from .constants import MU_EARTH, OMEGA_EARTH, G0
from .quaternion import (
    quat_derivative,
    quat_normalize,
    quat_between_vectors,
    rotate_body_to_eci,
    rotate_eci_to_body,
)

STATE_SIZE = 14


@dataclass
class Environment:
    altitude: float
    rho: float
    pressure: float
    temperature: float
    speed_of_sound: float
    v_rel_eci: np.ndarray
    v_rel_body: np.ndarray
    v_rel_mag: float
    mach: float
    alpha_total: float   # total angle of attack, rad
    q_dynamic: float      # dynamic pressure, Pa


def compute_environment(r_eci: np.ndarray, v_eci: np.ndarray, q_att: np.ndarray) -> Environment:
    altitude = float(np.linalg.norm(r_eci) - 6371000.0)
    rho, pressure, temperature, a_sound = atmosphere(altitude)

    omega_vec = np.array([0.0, 0.0, OMEGA_EARTH])
    v_atm_eci = np.cross(omega_vec, r_eci)
    v_rel_eci = v_eci - v_atm_eci
    v_rel_body = rotate_eci_to_body(q_att, v_rel_eci)
    v_rel_mag = float(np.linalg.norm(v_rel_body))
    mach = v_rel_mag / a_sound if a_sound > 0 else 0.0

    if v_rel_mag > 1e-6:
        alpha_total = float(np.arccos(np.clip(v_rel_body[0] / v_rel_mag, -1.0, 1.0)))
    else:
        alpha_total = 0.0

    q_dynamic = 0.5 * rho * v_rel_mag ** 2

    return Environment(
        altitude=altitude, rho=rho, pressure=pressure, temperature=temperature,
        speed_of_sound=a_sound, v_rel_eci=v_rel_eci, v_rel_body=v_rel_body,
        v_rel_mag=v_rel_mag, mach=mach, alpha_total=alpha_total, q_dynamic=q_dynamic,
    )


def aero_forces_moments(env: Environment, aero, omega_body: np.ndarray):
    """Aerodynamic force (body frame, N) and moment (body frame, N*m)."""
    if env.v_rel_mag < 1e-6 or env.q_dynamic < 1e-9:
        return np.zeros(3), np.zeros(3)

    v_body = env.v_rel_body
    v_hat = v_body / env.v_rel_mag
    alpha = env.alpha_total
    # sin(alpha) rather than alpha itself bounds every coefficient build-up
    # below to a sane, periodic range over the FULL 0-180 degree total
    # angle-of-attack domain (needed once the vehicle is uncontrolled and
    # free to tumble in coast/re-entry) -- a bare linear-in-alpha model
    # blows up unphysically past small angles.
    sin_a = math.sin(alpha)

    cd = aero.cd0(env.mach) + aero.k_alpha * sin_a ** 2
    cn = aero.cl_alpha * sin_a

    drag_mag = env.q_dynamic * aero.ref_area * cd
    drag_force = -drag_mag * v_hat

    # Normal (lift) force must be perpendicular to the RELATIVE WIND, not to
    # the body axis -- a real aerodynamic normal force does zero work on the
    # vehicle (F . v_rel = 0) because it is defined perpendicular to the
    # flow; only drag (anti-parallel to the flow) may remove kinetic energy.
    # The standard construction (e.g. Zipfel, "Modeling and Simulation of
    # Aerospace Vehicle Dynamics") is the component of the body x-axis
    # perpendicular to the relative wind -- NOT the component of the
    # relative wind perpendicular to the body x-axis, which is a different,
    # non-perpendicular-to-flow vector that has a spurious component ALONG
    # the flow and so silently injects energy into the vehicle every step
    # once alpha moves away from 0/90/180 degrees (exactly what an
    # uncontrolled tumbling re-entry does), causing runaway divergence.
    x_axis = np.array([1.0, 0.0, 0.0])
    normal_dir = x_axis - np.dot(x_axis, v_hat) * v_hat
    n_norm = np.linalg.norm(normal_dir)
    if n_norm > 1e-9:
        normal_dir = normal_dir / n_norm
        lift_force = env.q_dynamic * aero.ref_area * cn * normal_dir
    else:
        lift_force = np.zeros(3)

    force_body = drag_force + lift_force

    # restoring/damping moment about the axis perpendicular to both the body
    # x-axis and the relative wind (i.e. the pitch/yaw plane torque axis)
    if n_norm > 1e-9:
        moment_axis = np.cross(x_axis, normal_dir)
        ma = np.linalg.norm(moment_axis)
        if ma > 1e-9:
            moment_axis = moment_axis / ma
    else:
        moment_axis = np.zeros(3)

    cm = aero.cm0 + aero.cm_alpha * sin_a
    moment_aero = env.q_dynamic * aero.ref_area * aero.ref_length * cm * moment_axis

    # damping moment opposes body angular rate about the pitch/yaw axes
    omega_pitchyaw = omega_body - np.dot(omega_body, x_axis) * x_axis
    if env.v_rel_mag > 1e-6:
        damping = -aero.cm_q * env.q_dynamic * aero.ref_area * (aero.ref_length ** 2) / (2 * env.v_rel_mag) * omega_pitchyaw
    else:
        damping = np.zeros(3)

    return force_body, moment_aero + damping


def attitude_control_torque(q_att: np.ndarray, omega_body: np.ndarray, target_dir_eci: Optional[np.ndarray],
                              max_torque: float, inertia_pitch: float,
                              response_time_s: float = 3.5, zeta: float = 0.9) -> np.ndarray:
    """Quaternion-feedback PD controller that points the body x-axis at
    `target_dir_eci`, saturated to +/- max_torque (an abstraction of TVC/fin
    control authority). Gains are scaled to the vehicle's own pitch/yaw
    inertia (critically-damped 2nd-order response with the given settling
    time) rather than fixed constants -- fixed gains would either be far too
    soft for a light/short vehicle or, worse, saturate-and-overshoot into a
    growing limit-cycle oscillation for a large one. Returns zero torque if
    there is no active guidance target."""
    if target_dir_eci is None or max_torque <= 0:
        return np.zeros(3)

    wn = 4.0 / max(response_time_s, 0.5)  # rad/s, ~4/tau for a well-damped settle
    kp = inertia_pitch * wn ** 2
    kd = 2.0 * zeta * inertia_pitch * wn

    x_body_eci = rotate_body_to_eci(q_att, np.array([1.0, 0.0, 0.0]))
    err_q = quat_between_vectors(x_body_eci, target_dir_eci)
    # small-angle axis*angle vector, expressed in ECI, then rotated to body
    err_vec_eci = 2.0 * err_q[1:4]
    err_vec_body = rotate_eci_to_body(q_att, err_vec_eci)

    torque = kp * err_vec_body - kd * omega_body
    norm = np.linalg.norm(torque)
    if norm > max_torque:
        torque = torque * (max_torque / norm)
    return torque


@dataclass
class FlightAux:
    """Auxiliary quantities recomputed each logged step, beyond the raw state."""
    env: Environment
    thrust_mag: float
    drag_mag: float
    lift_mag: float
    control_torque: np.ndarray
    mass_flow: float
    powered: bool


def state_derivative(t: float, state: np.ndarray, mass_flow: float, thrust_mag: float,
                       aero, inertia_diag: np.ndarray, target_dir_eci, max_control_torque: float):
    r = state[0:3]
    v = state[3:6]
    q = quat_normalize(state[6:10])
    omega = state[10:13]
    mass = max(state[13], 1e-3)

    env = compute_environment(r, v, q)

    # gravity (point mass, ECI is inertial -> no fictitious forces)
    r_norm = np.linalg.norm(r)
    g_eci = -MU_EARTH * r / (r_norm ** 3)

    force_aero_body, moment_aero_body = aero_forces_moments(env, aero, omega)

    thrust_body = np.array([thrust_mag, 0.0, 0.0])
    force_body_total = force_aero_body + thrust_body
    force_eci = rotate_body_to_eci(q, force_body_total)

    a_eci = force_eci / mass + g_eci

    ixx, iyy = inertia_diag
    izz = iyy
    I = np.array([ixx, iyy, izz])

    control_torque = attitude_control_torque(q, omega, target_dir_eci, max_control_torque, iyy)
    moment_total = moment_aero_body + control_torque

    omega_dot = (moment_total - np.cross(omega, I * omega)) / I

    qdot = quat_derivative(q, omega)

    dstate = np.zeros(STATE_SIZE)
    dstate[0:3] = v
    dstate[3:6] = a_eci
    dstate[6:10] = qdot
    dstate[10:13] = omega_dot
    dstate[13] = -mass_flow

    aux = FlightAux(
        env=env, thrust_mag=thrust_mag, drag_mag=float(np.linalg.norm(force_aero_body[0])),
        lift_mag=float(np.linalg.norm(force_aero_body[1:])), control_torque=control_torque,
        mass_flow=mass_flow, powered=thrust_mag > 0.0,
    )
    return dstate, aux
