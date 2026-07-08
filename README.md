# 6DOF Ballistic Missile Range

A six-degree-of-freedom (6DOF) rigid-body flight-dynamics simulator for
ballistic missile / sounding-rocket-class trajectories, with a Streamlit
GUI, an analysis module, and hand-built animated SVG output graphs.

**[Launch the live app &rarr;](https://sixdof-missile-range.streamlit.app/)**
- no install needed, runs in your browser via Streamlit Community Cloud.

**New to running it yourself?** Jump to
[Quick Start](#quick-start-zero-experience-needed) below - it walks
through everything from "I have never used a terminal" to a running
simulation, step by step. There's also a [GitHub Wiki](../../wiki) with
the same guide plus deeper how-tos, and a static
[flight-report demo](https://timeout187.github.io/ballistic-missile-range/)
hosted on GitHub Pages if you just want to see it animate without running
anything.

---

## Quick Start (zero experience needed)

This walks through everything needed to get the simulator running on your
own computer, assuming you have never used a terminal before. It takes
about 5 minutes. Every command below is something you type exactly as
written, then press Enter.

### What you need

- A Windows, Mac, or Linux computer.
- An internet connection (just for the one-time setup).
- That's it - no coding experience required to *run* it.

### Step 1: Check if Python is already installed

Python is the programming language this project is written in. Most of the
setup below happens through a **terminal** (a text window where you type
commands instead of clicking icons).

**Open a terminal:**
- **Windows**: press the Windows key, type `PowerShell`, press Enter.
- **Mac**: press `Cmd + Space`, type `Terminal`, press Enter.

Type this and press Enter:

```bash
python --version
```

- If you see something like `Python 3.11.0`, you already have Python -
  skip to Step 2.
- If you see an error (`command not found`, or Windows offers to open the
  Microsoft Store), you need to install Python first: go to
  [python.org/downloads](https://www.python.org/downloads/), download the
  latest version, run the installer. **On Windows, make sure to check the
  box that says "Add Python to PATH"** during install - it's easy to miss
  and things won't work without it. Restart your terminal after installing.

### Step 2: Download this project

If you have `git` installed, run:

```bash
git clone https://github.com/timeout187/ballistic-missile-range.git
cd ballistic-missile-range
```

Don't have `git` or don't know what that means? On this repository's GitHub
page, click the green **Code** button, then **Download ZIP**. Unzip it
anywhere (e.g. your Desktop), then in your terminal navigate into that
folder - for example:

```bash
cd Desktop/ballistic-missile-range
```

### Step 3: Install the project's dependencies

This downloads the handful of code libraries the simulator relies on
(numerical math, the web GUI framework, the charting library). Run:

```bash
pip install -r requirements.txt
```

This can take a minute or two the first time. If `pip` isn't recognized,
try `pip3` or `python -m pip install -r requirements.txt` instead.

### Step 4: Launch the simulator

```bash
streamlit run app.py
```

A browser tab should open automatically at `http://localhost:8501` showing
the app. If it doesn't open by itself, copy that address into your browser
manually. **Leave the terminal window open** - closing it stops the app.

### Step 5: Run your first simulation

1. On the left sidebar, leave the default **Preset** ("Russia - Scud-B")
   selected - it's a ready-to-go example.
2. Scroll down and click the red **Run Simulation** button.
3. Wait a few seconds - the physics engine is integrating the full flight,
   launch to impact.
4. You'll see the flight results appear: apogee, downrange distance, max
   speed, and more, followed by tabs. Click **Animated Flight Report
   (SVG)**, then press the **Play** button to watch the simulated flight
   animate in real time, with live altitude/speed/stress charts scrolling
   alongside it.

That's it - you're running a real 6DOF aerospace simulation. From here,
try a different preset in the sidebar, or expand **Stages** / **Payload /
re-entry body** to change the vehicle's mass, thrust, or aerodynamics, and
click **Run Simulation** again to see how the flight changes.

### Stopping the app

Go back to the terminal window and press `Ctrl + C`. Closing the terminal
window also stops it.

### Troubleshooting

| Problem | Fix |
|---|---|
| `python: command not found` | Python isn't installed, or wasn't added to PATH - see Step 1. On some systems the command is `python3` instead of `python`. |
| `pip: command not found` | Try `pip3` or `python -m pip install -r requirements.txt`. |
| `streamlit: command not found` | The install in Step 3 didn't finish or failed - re-run `pip install -r requirements.txt` and check for red error text above. |
| Browser tab doesn't open | Manually open `http://localhost:8501` in any browser while the terminal is still running. |
| "This vehicle cannot lift off" warning | The thrust you configured is less than the vehicle's weight - increase thrust or reduce mass in the sidebar's Stages panel, or just pick a different preset. |
| Port already in use | Another program is using port 8501. Run `streamlit run app.py --server.port 8502` instead, then open `http://localhost:8502`. |
| Still stuck | Open an [issue on GitHub](../../issues) with what you typed and what error you saw. |

For everything beyond this - what each tab means, how the physics works,
how to add your own vehicle - see the [Wiki](../../wiki).

---

This project began as a fork of a 2005, Python 2/wxPython, **planar**
(range-and-altitude-only, no rotation) ICBM trajectory tool written by Josh
Levinger for GlobalSecurity.org, based on Dr. David Wright's (MIT) 1992
BASIC model, with later GUI/packaging updates by Karsten Wolf
([@karstenw](https://github.com/karstenw)). That original tool is
preserved for reference, with full credit, in
[`legacy_2005_planar_tool/`](legacy_2005_planar_tool/) - including its own
copyright notice in [`LICENSE`](LICENSE), which stays in force regardless
of the upstream repository's archived status. Everything below is new: a
full 3D translational **and** rotational rigid-body simulation, a modern
GUI, and an animated data-analysis output module.

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

Or with Docker, no Python setup at all - a prebuilt image is published to
GitHub Container Registry on every release:

```bash
docker run -p 8501:8501 ghcr.io/timeout187/ballistic-missile-range:latest
```

Then open `http://localhost:8501`. (First pull downloads the image; after
that, `docker run` is instant.)

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

### Python version

Targets **Python 3.14** (the current latest stable release, security-
supported through October 2030) - pinned in the Docker image, CI, and
`runtime.txt` (read by Streamlit Community Cloud). Should also run on any
3.10+ interpreter if you're installing manually.

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
- **Historical preset data**: copied verbatim from the original
  `legacy_2005_planar_tool/presets.txt` - stage masses, Isp, thrust and
  diameters are used exactly as sourced, with no modification. The sheet's
  thrust is in kgf (kilogram-force) and is converted to Newtons with the
  same `*9.81` the original GUI used; the single Isp the sheet lists per
  stage is used at all altitudes. The only per-stage values not in the
  sheet - length (for the rotational-inertia estimate) and the aerodynamic
  coefficients - are documented engineering estimates that the 2D original
  never needed, and do not alter the ballistic data. See the comments in
  `sixdof/presets.py`.
- **Presets are illustrative**, not intelligence-grade specifications.

## Extending it

- Swap in real aerodynamic coefficient tables (Mach x alpha lookup) in
  `StageAero` / `aero_forces_moments()`.
- Add a J2 gravity term or wind model in `dynamics.py`.
- Add a closed-loop guidance/targeting solver in `guidance.py`.
- The physics core (`sixdof/`, `analysis/`) has no GUI dependency, so it is
  directly usable from a notebook or a batch script for parameter sweeps -
  see the docstrings in `sixdof/simulation.py`.

## Credits

- **Josh Levinger** - wrote the original 2005 planar range/altitude tool
  for GlobalSecurity.org.
- **Dr. David Wright (MIT)** - the underlying trajectory physics are based
  on his 1992 BASIC model, published in "Depressed Trajectory SLBMs,"
  *Science and Global Security*, Vol 3, p101-159.
- **Karsten Wolf** ([@karstenw](https://github.com/karstenw)) - GUI and
  packaging updates to the original tool.
- **Hasan Ahmed** - six-degrees-of-freedom rigid-body rewrite: the
  `sixdof/` physics engine, `analysis/` module, Streamlit GUI, animated
  SVG flight report, and everything else added since the fork.

See [`LICENSE`](LICENSE) for the full copyright notice (MIT, covering
both the original 2013 copyright and the 2026 rewrite).
