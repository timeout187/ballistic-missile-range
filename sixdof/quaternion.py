"""
Quaternion attitude helpers. Convention: q = [q0, q1, q2, q3] scalar-first,
representing the rotation FROM body frame TO the inertial (ECI) frame, i.e.
v_eci = R(q) @ v_body.
"""

from __future__ import annotations

import numpy as np


def quat_normalize(q: np.ndarray) -> np.ndarray:
    n = np.linalg.norm(q)
    if n < 1e-12:
        return np.array([1.0, 0.0, 0.0, 0.0])
    return q / n


def quat_mult(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    a0, a1, a2, a3 = a
    b0, b1, b2, b3 = b
    return np.array([
        a0 * b0 - a1 * b1 - a2 * b2 - a3 * b3,
        a0 * b1 + a1 * b0 + a2 * b3 - a3 * b2,
        a0 * b2 - a1 * b3 + a2 * b0 + a3 * b1,
        a0 * b3 + a1 * b2 - a2 * b1 + a3 * b0,
    ])


def quat_conj(q: np.ndarray) -> np.ndarray:
    return np.array([q[0], -q[1], -q[2], -q[3]])


def quat_from_axis_angle(axis: np.ndarray, angle: float) -> np.ndarray:
    axis = np.asarray(axis, dtype=float)
    n = np.linalg.norm(axis)
    if n < 1e-12:
        return np.array([1.0, 0.0, 0.0, 0.0])
    axis = axis / n
    half = angle / 2.0
    s = np.sin(half)
    return np.array([np.cos(half), axis[0] * s, axis[1] * s, axis[2] * s])


def quat_to_dcm(q: np.ndarray) -> np.ndarray:
    """Rotation matrix (body -> inertial): v_eci = dcm @ v_body."""
    q0, q1, q2, q3 = quat_normalize(q)
    return np.array([
        [1 - 2 * (q2 ** 2 + q3 ** 2), 2 * (q1 * q2 - q0 * q3), 2 * (q1 * q3 + q0 * q2)],
        [2 * (q1 * q2 + q0 * q3), 1 - 2 * (q1 ** 2 + q3 ** 2), 2 * (q2 * q3 - q0 * q1)],
        [2 * (q1 * q3 - q0 * q2), 2 * (q2 * q3 + q0 * q1), 1 - 2 * (q1 ** 2 + q2 ** 2)],
    ])


def rotate_body_to_eci(q: np.ndarray, v_body: np.ndarray) -> np.ndarray:
    return quat_to_dcm(q) @ v_body


def rotate_eci_to_body(q: np.ndarray, v_eci: np.ndarray) -> np.ndarray:
    return quat_to_dcm(q).T @ v_eci


def quat_derivative(q: np.ndarray, omega_body: np.ndarray) -> np.ndarray:
    """dq/dt for q(t) representing body->inertial, given body-frame angular rate."""
    wx, wy, wz = omega_body
    omega_quat = np.array([0.0, wx, wy, wz])
    return 0.5 * quat_mult(q, omega_quat)


def euler_from_quat(q: np.ndarray):
    """321 (yaw-pitch-roll) Euler angles, radians, from body->inertial-style
    quaternion applied relative to whatever reference frame q is expressed in
    (caller is responsible for passing a quaternion relative to the desired
    reference, e.g. local NED). Returns (roll, pitch, yaw)."""
    q0, q1, q2, q3 = quat_normalize(q)

    sinr_cosp = 2 * (q0 * q1 + q2 * q3)
    cosr_cosp = 1 - 2 * (q1 ** 2 + q2 ** 2)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (q0 * q2 - q3 * q1)
    sinp = np.clip(sinp, -1.0, 1.0)
    pitch = np.arcsin(sinp)

    siny_cosp = 2 * (q0 * q3 + q1 * q2)
    cosy_cosp = 1 - 2 * (q2 ** 2 + q3 ** 2)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def quat_between_vectors(v_from: np.ndarray, v_to: np.ndarray) -> np.ndarray:
    """Shortest-arc quaternion rotating v_from onto v_to (both need not be unit)."""
    v_from = v_from / (np.linalg.norm(v_from) + 1e-15)
    v_to = v_to / (np.linalg.norm(v_to) + 1e-15)
    dot = np.clip(np.dot(v_from, v_to), -1.0, 1.0)
    if dot > 1 - 1e-9:
        return np.array([1.0, 0.0, 0.0, 0.0])
    if dot < -1 + 1e-9:
        # 180 degree case: pick any orthogonal axis
        axis = np.cross(v_from, np.array([1.0, 0.0, 0.0]))
        if np.linalg.norm(axis) < 1e-6:
            axis = np.cross(v_from, np.array([0.0, 1.0, 0.0]))
        return quat_from_axis_angle(axis, np.pi)
    axis = np.cross(v_from, v_to)
    angle = np.arccos(dot)
    return quat_from_axis_angle(axis, angle)
