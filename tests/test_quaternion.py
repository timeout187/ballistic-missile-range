import numpy as np
import pytest

from sixdof.quaternion import (
    euler_from_quat,
    quat_between_vectors,
    quat_conj,
    quat_derivative,
    quat_from_axis_angle,
    quat_mult,
    quat_normalize,
    quat_to_dcm,
    rotate_body_to_eci,
    rotate_eci_to_body,
)

IDENTITY = np.array([1.0, 0.0, 0.0, 0.0])


def test_identity_quaternion_is_no_rotation():
    v = np.array([1.0, 2.0, 3.0])
    assert np.allclose(rotate_body_to_eci(IDENTITY, v), v)


def test_normalize_handles_near_zero_and_unit_norm():
    assert np.allclose(quat_normalize(np.zeros(4)), IDENTITY)
    q = quat_normalize(np.array([2.0, 0.0, 0.0, 0.0]))
    assert np.linalg.norm(q) == pytest.approx(1.0)


def test_quat_mult_identity_is_neutral_element():
    q = quat_normalize(np.array([0.5, 0.5, 0.5, 0.5]))
    assert np.allclose(quat_mult(q, IDENTITY), q)
    assert np.allclose(quat_mult(IDENTITY, q), q)


def test_conjugate_of_unit_quaternion_is_inverse():
    q = quat_from_axis_angle(np.array([0.0, 0.0, 1.0]), np.pi / 3)
    result = quat_mult(q, quat_conj(q))
    assert np.allclose(result, IDENTITY, atol=1e-9)


def test_90deg_rotation_about_z_maps_x_to_y():
    q = quat_from_axis_angle(np.array([0.0, 0.0, 1.0]), np.pi / 2)
    v_eci = rotate_body_to_eci(q, np.array([1.0, 0.0, 0.0]))
    assert np.allclose(v_eci, [0.0, 1.0, 0.0], atol=1e-9)


def test_rotate_eci_to_body_is_inverse_of_body_to_eci():
    q = quat_from_axis_angle(np.array([1.0, 1.0, 0.0]), 0.7)
    v_body = np.array([0.3, -0.8, 2.1])
    v_eci = rotate_body_to_eci(q, v_body)
    v_back = rotate_eci_to_body(q, v_eci)
    assert np.allclose(v_back, v_body, atol=1e-9)


def test_dcm_is_orthonormal():
    q = quat_from_axis_angle(np.array([0.2, 0.7, -0.3]), 1.234)
    dcm = quat_to_dcm(q)
    assert np.allclose(dcm @ dcm.T, np.eye(3), atol=1e-9)
    assert np.linalg.det(dcm) == pytest.approx(1.0, abs=1e-9)


def test_quat_between_vectors_maps_from_onto_to():
    v_from = np.array([1.0, 0.0, 0.0])
    v_to = np.array([0.0, 0.0, 1.0])
    q = quat_between_vectors(v_from, v_to)
    result = rotate_body_to_eci(q, v_from)
    assert np.allclose(result / np.linalg.norm(result), v_to, atol=1e-9)


def test_quat_between_parallel_vectors_is_identity():
    v = np.array([2.0, -1.0, 0.5])
    q = quat_between_vectors(v, v)
    assert np.allclose(q, IDENTITY, atol=1e-9)


def test_quat_between_antiparallel_vectors_is_valid_180deg_rotation():
    v_from = np.array([1.0, 0.0, 0.0])
    v_to = np.array([-1.0, 0.0, 0.0])
    q = quat_between_vectors(v_from, v_to)
    result = rotate_body_to_eci(q, v_from)
    assert np.allclose(result / np.linalg.norm(result), v_to, atol=1e-6)


def test_quat_derivative_zero_rate_gives_zero_derivative():
    q = quat_from_axis_angle(np.array([0.0, 1.0, 0.0]), 0.4)
    dq = quat_derivative(q, np.array([0.0, 0.0, 0.0]))
    assert np.allclose(dq, np.zeros(4))


def test_euler_from_identity_quaternion_is_zero():
    roll, pitch, yaw = euler_from_quat(IDENTITY)
    assert roll == pytest.approx(0.0, abs=1e-9)
    assert pitch == pytest.approx(0.0, abs=1e-9)
    assert yaw == pytest.approx(0.0, abs=1e-9)
