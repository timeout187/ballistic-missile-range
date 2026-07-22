# Equations of Motion

This page is the mathematical reference for the 6DOF Ballistic Missile
Range simulator: the state vector, the forces/moments driving it, and
where each equation lives in the code. It complements the higher-level
description in the [README](../README.md#what-6dof-adds-over-the-original)
and the design-notes comments in [`sixdof/dynamics.py`](../sixdof/dynamics.py).

## Nomenclature

| Symbol | Meaning |
|---|---|
| `r`, `v` | position, velocity, Earth-Centered Inertial (ECI) frame, m / m&middot;s&#8315;&sup1; |
| `q = [q0 q1 q2 q3]` | attitude quaternion, scalar-first, body &rarr; ECI |
| `omega` | body-frame angular rate, rad&middot;s&#8315;&sup1; |
| `m` | vehicle mass, kg |
| `alpha` | total angle of attack (angle between body x-axis and relative wind), rad |
| `q_dyn` | dynamic pressure, &frac12; &rho; &#124;v_rel&#124;&sup2;, Pa |
| `Cd`, `CN`, `Cm` | axial-force, normal-force, and pitch/yaw-moment coefficients |
| `I = diag(Ixx, Iyy, Izz)` | body-frame inertia tensor (`Iyy = Izz`, axisymmetric) |
| `mu` | Earth gravitational parameter, m&sup3;&middot;s&#8315;&sup2; |
| `omega_E` | Earth rotation rate, rad&middot;s&#8315;&sup1; |

## State vector (14 elements)

Integrated in the ECI frame by [`sixdof/simulation.py`](../sixdof/simulation.py)
via `scipy.integrate.solve_ivp` (adaptive RK45):

```
state[0:3]   r_eci             position, m
state[3:6]   v_eci             velocity, m/s
state[6:10]  q                 attitude quaternion (body -> ECI)
state[10:13] omega_body        body-frame angular rate, rad/s
state[13]    m                 vehicle mass, kg
```

Gravity is a point-mass central force, so the ECI frame is genuinely
inertial for translation - no centrifugal/Coriolis terms are needed
there. The atmosphere still co-rotates with the Earth, so aerodynamic
quantities use the velocity *relative to the local air mass*:

```
v_rel = v_eci - omega_E x r_eci,   omega_E = (0, 0, omega_E)
```

See [`compute_environment()`](../sixdof/dynamics.py) for the exact
implementation.

## Translational dynamics

```
dv/dt = (F_thrust + F_aero) / m  +  g(r)
```

with point-mass gravity

```
g(r) = -mu * r / |r|^3
```

(`sixdof/constants.py` for `mu`; no J2 oblateness term - see
[Assumptions and limitations](../README.md#physics--modeling-notes--scope)).

## Rotational dynamics (Euler's equations, axisymmetric body)

The vehicle is treated as axisymmetric (`Iyy = Izz`), so the general
rigid-body law `I * omega_dot + omega x (I * omega) = M_total` reduces
to the same closed form used by both sibling projects in this series:

```
omega_dot = (M_total - omega x (I * omega)) / I     (component-wise, I = diag(Ixx, Iyy, Izz))
```

implemented directly in [`state_derivative()`](../sixdof/dynamics.py).
`Ixx`, `Iyy` come from `Vehicle.inertia_estimate()` (uniform slender
cylinder approximation - see
[Assumptions and limitations](../README.md#physics--modeling-notes--scope)).

Attitude is propagated as a quaternion, not Euler angles (no gimbal-lock
singularity, valid through the full tumble a re-entering, uncontrolled
body can undergo):

```
dq/dt = 1/2 * q (x) [0, omega]     ((x) = quaternion multiplication)
```

see [`quat_derivative()`](../sixdof/quaternion.py).

## Aerodynamic forces and moments

Given dynamic pressure `q_dyn`, reference area `A`, reference length
`d` (diameter), and total angle of attack `alpha` (the angle between
the body x-axis and the relative wind, valid over the *full* 0-180&deg;
range, not just the small-angle region):

```
Cd            = Cd0(Mach) + k_alpha * sin^2(alpha)
CN            = CL_alpha * sin(alpha)

Drag          = q_dyn * A * Cd                      (anti-parallel to v_rel)
Normal force  = q_dyn * A * CN                       (perpendicular to v_rel, in the alpha-plane)

Cm            = Cm0 + Cm_alpha * sin(alpha)
Restoring
  moment      = q_dyn * A * d * Cm                  (about the pitch/yaw torque axis)
Damping
  moment      = -Cm_q * q_dyn * A * d^2 / (2|v_rel|) * omega_pitchyaw
```

**Why `sin(alpha)` and not `alpha` itself**: a linear-in-alpha model is
only valid for small angles. Once guidance authority is withdrawn at
burnout, an unstable body is free to tumble through the *entire* 0-180&deg;
range - `sin(alpha)` keeps every coefficient bounded and periodic there
instead of diverging. See the comment above `aero_forces_moments()` in
[`sixdof/dynamics.py`](../sixdof/dynamics.py) for the derivation of why
the normal force must be built from the component of the **body x-axis**
perpendicular to the relative wind, not the reverse (the naive
reversed construction silently injects energy into the vehicle once
alpha moves away from 0/90/180&deg;, which is exactly what a tumbling
re-entry does, and causes runaway numerical divergence).

`Cd0(Mach)` itself is a three-regime, continuous blend - subsonic
plateau, a sine-shaped transonic rise peaking at the transonic
coefficient, then an exponential decay toward the supersonic
coefficient - see `StageAero.cd0()` in
[`sixdof/vehicle.py`](../sixdof/vehicle.py).

## Attitude control (guidance-active phases only)

A quaternion-feedback PD controller points the body x-axis at the
guidance-commanded direction, critically damped and scaled to the
vehicle's own pitch/yaw inertia, saturated to the stage's control
authority:

```
torque = kp * err_vec_body - kd * omega_body,   saturated to +/- max_torque
kp = Iyy * wn^2,   kd = 2 * zeta * Iyy * wn,   wn = 4 / response_time_s
```

where `err_vec_body` is the (small-angle) axis*angle rotation vector
from the current body x-axis to the target direction. See
`attitude_control_torque()` in
[`sixdof/dynamics.py`](../sixdof/dynamics.py). **This torque is exactly
zero once guidance authority is withdrawn at final-stage burnout** -
verified by [`tests/test_simulation.py::test_guidance_authority_withdrawn_after_burnout`](../tests/test_simulation.py).

## Atmosphere

1976 U.S. Standard Atmosphere, piecewise-linear/isothermal through the
8 official layers to 86 km, with a smooth exponential tail above that
(see [`sixdof/atmosphere.py`](../sixdof/atmosphere.py) and
`tests/test_atmosphere.py` for the full layer table and validation
against the sea-level standard values).

## Guidance / pitch programs

Three open-loop boost profiles (`GuidanceProgram.target_direction()` in
[`sixdof/guidance.py`](../sixdof/guidance.py)): a vertical rise, a
pitch-over kick, then either prograde-lock (`gravity_turn`) or a held
constant inertial pitch angle (`fixed_pitch` / `depressed`). The
theoretical minimum-energy burnout flight-path angle for a given target
range is available as a diagnostic via `minimum_energy_gamma()`, but is
not closed-loop - see the docstring and
[`tests/test_guidance.py`](../tests/test_guidance.py) for its exact
closed-form definition.
