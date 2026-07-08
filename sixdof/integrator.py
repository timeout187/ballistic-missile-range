"""Fixed-step classical RK4 integrator for the 14-state 6DOF vector."""

from __future__ import annotations

import numpy as np

from .quaternion import quat_normalize


def rk4_step(deriv_fn, t: float, state: np.ndarray, dt: float):
    """deriv_fn(t, state) -> dstate (numpy array). Returns the new state."""
    k1 = deriv_fn(t, state)
    k2 = deriv_fn(t + dt / 2.0, state + dt / 2.0 * k1)
    k3 = deriv_fn(t + dt / 2.0, state + dt / 2.0 * k2)
    k4 = deriv_fn(t + dt, state + dt * k3)
    new_state = state + (dt / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)
    new_state[6:10] = quat_normalize(new_state[6:10])
    return new_state
