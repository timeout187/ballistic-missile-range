# 6DOF Ballistic Missile Range

A six-degree-of-freedom (6DOF) rigid-body flight-dynamics simulator for
ballistic missile / sounding-rocket-class trajectories, with a Streamlit
GUI, an analysis module, and hand-built animated SVG output graphs.

This project began as a fork of a 2005, Python 2/wxPython, **planar**
(range-and-altitude-only, no rotation) ICBM trajectory tool written by Josh
Levinger for GlobalSecurity.org, based on Dr. David Wright's (MIT) 1992
BASIC model. That original tool is preserved for reference in
[`legacy_2005_planar_tool/`](legacy_2005_planar_tool/). Everything below is
new: a full 3D translational **and** rotational rigid-body simulation, a
modern GUI, and an animated data-analysis output module.

## What "6DOF" adds over the original

The original model integrated two numbers per instant - range and altitude
- along a single vertical plane, with the flight-path angle set kinematically
by a hand-written control law. It could never show a vehicle tumble, weathercock,
or respond to a real disturbance, because it had no attitude state at all.

This version integrates a full 14-element rigid-body state:

- **3D translation** in an Earth-Centered Inertial (ECI) frame under
  point-mass gravity (position + velocity, 6 states).
- **3D rotation** via a unit quaternion + body-frame angular rate (7 states),
  driven by Euler's rigid-body equations with a per-vehicle inertia tensor.
- **Mass** as the 14th state, decreasing with propellant consumption.

Forces and moments come from: point-mass gravity, a Mach/angle-of-attack
dependent aerodynamic model (drag, normal force, and a restoring/damping
pitch-yaw moment, valid over the *full* 0-180 degree total angle-of-attack
range so a tumbling re-entry doesn't blow up numerically), motor thrust, and
a torque-saturated attitude controller (an abstraction of TVC/fin actuation)
that is only active while a stage has guidance authority. **The moment
control authority is withdrawn entirely at final-stage burnout** - the
vehicle then coasts and re-enters under aerodynamics and inertia alone,
free to weathercock (if statically stable) or tumble (if not). That
behavior, and the resulting swings in dynamic pressure, angle of attack,
and axial load during re-entry, is the entire point of doing this in 6DOF -
the old planar model could not represent it at all.

See [`sixdof/dynamics.py`](sixdof/dynamics.py) and
[`sixdof/guidance.py`](sixdof/guidance.py) for the full equations and the
guidance-law design notes.

## Project layout

```
sixdof/            physics engine (no GUI dependency)
  constants.py        physical constants
  atmosphere.py        1976 U.S. Standard Atmosphere
  quaternion.py         attitude math
  vehicle.py             Stage / StageAero / Vehicle dataclasses
  presets.py               ported + corrected historical missile presets
  guidance.py                open-loop pitch program / attitude targeting
  dynamics.py                   forces, moments, the 14-state derivative
  integrator.py                  fixed-step RK4
  simulation.py                    orchestrates staging, boost, coast, re-entry

analysis/           post-processing (no GUI dependency)
  metrics.py           apogee, max-Q, max Mach, max load, impact point, stability
  export.py             CSV / JSON export
  svg_report.py           hand-built animated SVG + vanilla-JS report generator

app.py              Streamlit GUI (sidebar inputs, tabs, downloads)
requirements.txt
legacy_2005_planar_tool/   the original 2005 planar tool, kept for reference
```

## Running it

```bash
pip install -r requirements.txt
streamlit run app.py
```

Opens a browser tab with:

- **Sidebar** - pick a preset vehicle (or edit every stage's mass, Isp,
  thrust, diameter, and aerodynamic coefficients directly), launch
  latitude/longitude/azimuth, a trajectory type (gravity turn / lofted
  minimum-energy / depressed), guidance timing, and numerical integration
  settings.
- **Animated Flight Report (SVG)** tab - the core data-analysis output: an
  altitude-vs-downrange profile and a ground track, each with a moving
  vehicle marker, plus a synced strip of animated mini-charts (altitude,
  Mach, dynamic pressure, angle of attack, axial load) - all plain SVG with
  a small vanilla-JS player/scrubber, not a plotting-library GIF/video.
- **3D Trajectory** tab - a bonus interactive WebGL view of the flight
  around a globe (Plotly), for a bird's-eye sanity check of the ECI path.
- **Analysis Summary** tab - apogee, burnout point, max Mach/max-Q/max
  load and when they occur, total range, impact point, and a static
  stability read-out per stage.
- **Raw Data / Export** tab - the full timestepped dataframe, downloadable
  as CSV or a metrics summary as JSON.

## Physics & modeling notes / scope

- **Atmosphere**: 1976 U.S. Standard Atmosphere, piecewise through 86 km,
  with a smooth exponential tail above that.
- **Gravity**: point-mass (`-mu r / |r|^3`); no J2 oblateness term.
- **Aerodynamics**: `Cd0(Mach)` (subsonic -> transonic peak -> supersonic
  decay) plus an angle-of-attack drag/lift/moment build-up expressed in
  `sin(alpha_total)` rather than raw `alpha`, so the model stays bounded and
  well-behaved even at the large angles of attack an uncontrolled, tumbling
  re-entry produces. This is a simplified, illustrative aerodynamic model,
  not one derived from wind-tunnel or CFD data - the coefficients are fully
  editable in the GUI's Advanced panel for anyone who wants to substitute
  better data.
- **Inertia**: estimated per stage as a uniform slender cylinder from mass,
  diameter, and length. Replace with vehicle-specific inertia data for
  research-grade fidelity.
- **Guidance**: open-loop. "Gravity turn" does a short vertical rise, a
  brief pitch-over kick, then commands zero angle of attack (prograde lock)
  for the rest of powered flight, letting gravity rotate the flight path -
  the classical technique. "Lofted / minimum-energy" and "depressed" hold a
  user-specified constant pitch angle after the kick instead. There is
  **no closed-loop range-targeting or impact-point-prediction solver** in
  this version (the original tool's Newton's-method fuel-fraction solver
  was not carried over); `sixdof.guidance.minimum_energy_gamma()` reports
  the classical optimal burnout angle for a given target range as a
  reference number, but guidance does not automatically fly to hit it.
- **Historical preset data**: ported from the original `presets.txt`.
  Its thrust figures are in kgf (kilogram-force), matching the original
  GUI's `*9.81` conversion - see the comments in `sixdof/presets.py` for two
  entries (Scud-B, Al-Husayn) whose sourced thrust was, even after that
  correction, below the vehicle's own liftoff weight, and were bumped to a
  flyable value close to commonly cited real-world figures.
- **Presets are illustrative**, not intelligence-grade specifications.

## Extending it

- Swap in real aerodynamic coefficient tables (Mach x alpha lookup) in
  `StageAero` / `aero_forces_moments()`.
- Add a J2 gravity term or wind model in `dynamics.py`.
- Add a closed-loop guidance/targeting solver in `guidance.py`.
- The physics core (`sixdof/`, `analysis/`) has no GUI dependency, so it is
  directly usable from a notebook or a batch script for parameter sweeps -
  see the docstrings in `sixdof/simulation.py`.
