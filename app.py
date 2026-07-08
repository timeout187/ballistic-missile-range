"""
Six-Degrees-of-Freedom Ballistic Missile / Rocket Trajectory Simulator
------------------------------------------------------------------------
Streamlit GUI on top of the `sixdof` rigid-body flight-dynamics engine.
Successor to the 2005 planar (range/altitude-only) tool this project began
as; see README.md for the physics and the scope of what changed.

Run with:  streamlit run app.py
"""

from __future__ import annotations

import copy
import math

import numpy as np
import pandas as pd
import streamlit as st
import streamlit.components.v1 as components

from sixdof.presets import PRESETS
from sixdof.vehicle import Vehicle, Stage, StageAero
from sixdof.guidance import GuidanceProgram, minimum_energy_gamma
from sixdof.simulation import Simulation
from analysis.metrics import compute_metrics, metrics_table
from analysis.export import dataframe_to_csv_bytes, metrics_to_json_bytes
from analysis.svg_report import build_animated_svg_report

st.set_page_config(page_title="6DOF Missile Range", layout="wide", page_icon="🚀")

# ---------------------------------------------------------------- styling --
st.markdown("""
<style>
.block-container { padding-top: 1.6rem; }
h1, h2, h3 { letter-spacing: .01em; }
.metric-note { color: #93a4c3; font-size: 13px; }
div[data-testid="stMetricValue"] { font-size: 1.35rem; }
</style>
""", unsafe_allow_html=True)

st.title("6DOF Ballistic Missile Range")
st.caption(
    "Six-degree-of-freedom rigid-body trajectory simulator - full 3D translation + rotation "
    "(quaternion attitude, aerodynamic moments, torque-limited attitude control, multi-stage "
    "boost, ballistic re-entry). Successor to the original 2005 planar range/altitude tool."
)

# ---------------------------------------------------------------- sidebar --
with st.sidebar:
    st.header("Vehicle")
    preset_name = st.selectbox("Preset", list(PRESETS.keys()), index=list(PRESETS.keys()).index("Russia - Scud-B"))

    if st.session_state.get("_preset_loaded") != preset_name:
        st.session_state["_vehicle_template"] = copy.deepcopy(PRESETS[preset_name])
        st.session_state["_preset_loaded"] = preset_name

    template: Vehicle = st.session_state["_vehicle_template"]

    with st.expander("Payload / re-entry body", expanded=False):
        payload_mass = st.number_input("Payload mass (kg)", 1.0, 1.0e5, float(template.payload_mass), step=10.0)
        payload_diam = st.number_input("Payload diameter (m)", 0.05, 10.0, float(template.payload_diameter), step=0.01)
        payload_cd0 = st.number_input("Payload Cd0", 0.05, 2.0, float(template.payload_cd0), step=0.01)
        payload_cl = st.number_input("Payload CL_alpha (/rad)", 0.0, 6.0, float(template.payload_cl_alpha), step=0.1)
        payload_cm = st.number_input(
            "Payload Cm_alpha (/rad) - negative = stable", -3.0, 3.0, float(template.payload_cm_alpha), step=0.05,
            help="Determines whether the re-entry body weathercocks (stable) or tumbles (unstable/neutral) once guidance authority is withdrawn at final burnout."
        )

    stage_forms = []
    with st.expander(f"Stages ({len(template.stages)})", expanded=False):
        for i, s in enumerate(template.stages):
            st.markdown(f"**Stage {i + 1}: {s.name}**")
            c1, c2 = st.columns(2)
            dry = c1.number_input(f"Dry mass (kg) #{i}", 1.0, 2.0e5, float(s.dry_mass), step=10.0, key=f"dry_{i}")
            prop = c2.number_input(f"Propellant mass (kg) #{i}", 1.0, 5.0e5, float(s.propellant_mass), step=10.0, key=f"prop_{i}")
            c3, c4 = st.columns(2)
            isp_sea = c3.number_input(f"Isp sea level (s) #{i}", 50.0, 500.0, float(s.isp_sea), step=1.0, key=f"ispsea_{i}")
            isp_vac = c4.number_input(f"Isp vacuum (s) #{i}", 50.0, 500.0, float(s.isp_vac), step=1.0, key=f"ispvac_{i}")
            thrust_kgf = st.number_input(
                f"Vacuum thrust (kgf) #{i}", 100.0, 5.0e6, float(s.thrust_vac / 9.81), step=100.0, key=f"thr_{i}",
                help="Entered in kgf (kilogram-force) for continuity with the historical preset data, converted internally to Newtons."
            )
            c5, c6 = st.columns(2)
            diam = c5.number_input(f"Diameter (m) #{i}", 0.05, 12.0, float(s.diameter), step=0.01, key=f"diam_{i}")
            length = c6.number_input(f"Length (m) #{i}", 0.5, 80.0, float(s.length), step=0.1, key=f"len_{i}")
            c7, c8 = st.columns(2)
            gimbal = c7.number_input(f"Max gimbal (deg) #{i}", 0.5, 20.0, float(s.gimbal_max_deg), step=0.5, key=f"gim_{i}")
            cm_alpha = c8.number_input(
                f"Cm_alpha (/rad) #{i}", -3.0, 3.0, float(s.aero.cm_alpha), step=0.05, key=f"cma_{i}",
                help="Powered-flight static stability; only matters if guidance authority saturates."
            )
            st.markdown("---")
            stage_forms.append(dict(
                name=s.name, dry=dry, prop=prop, isp_sea=isp_sea, isp_vac=isp_vac,
                thrust_kgf=thrust_kgf, diam=diam, length=length, gimbal=gimbal, cm_alpha=cm_alpha,
            ))

    st.header("Launch")
    c1, c2 = st.columns(2)
    launch_lat = c1.number_input("Launch latitude (deg)", -89.0, 89.0, 35.0, step=0.5)
    launch_lon = c2.number_input("Launch longitude (deg)", -180.0, 180.0, 45.0, step=0.5)
    azimuth = st.slider("Launch azimuth (deg, 0=N, 90=E)", 0.0, 359.9, 90.0, step=1.0)

    st.header("Guidance / trajectory")
    traj_type = st.radio(
        "Trajectory type", ["gravity_turn", "fixed_pitch", "depressed"],
        format_func=lambda v: {"gravity_turn": "Gravity turn (standard)", "fixed_pitch": "Lofted / minimum-energy (fixed pitch)", "depressed": "Depressed trajectory"}[v],
    )
    with st.expander("Guidance parameters", expanded=False):
        t_vertical = st.number_input("Vertical rise time (s)", 0.0, 30.0, 6.0, step=0.5)
        t_pitch = st.number_input("Pitch-over kick duration (s)", 1.0, 40.0, 10.0, step=0.5)
        pitch_kick = st.number_input("Pitch-over kick angle (deg)", 0.5, 20.0, 3.0, step=0.5)
        boost_pitch = 45.0
        if traj_type != "gravity_turn":
            default_pitch = 45.0 if traj_type == "fixed_pitch" else 20.0
            boost_pitch = st.slider(
                "Boost pitch angle held after kick (deg from local horizontal)",
                5.0, 89.0, default_pitch, step=1.0,
            )
            target_range_km = st.number_input("Reference target range for min-energy angle (km)", 50.0, 15000.0, 3000.0, step=50.0)
            phi = target_range_km * 1000.0 / 6371000.0
            gamma_opt = math.degrees(minimum_energy_gamma(phi))
            st.caption(f"Classical minimum-energy burnout angle for {target_range_km:.0f} km range: **{gamma_opt:.1f} deg** (diagnostic only - not auto-applied).")

    st.header("Numerical integration")
    with st.expander("Advanced numerics", expanded=False):
        dt_boost = st.number_input("Boost/dense-atmosphere timestep (s)", 0.005, 0.2, 0.02, step=0.005, format="%.3f")
        dt_coast = st.number_input("Exoatmospheric coast timestep (s)", 0.05, 2.0, 0.25, step=0.05)
        t_max = st.number_input("Max simulated flight time (s)", 60.0, 7200.0, 3600.0, step=60.0)

    run_clicked = st.button("Run Simulation", type="primary", use_container_width=True)


def build_vehicle() -> Vehicle:
    stages = []
    for sf in stage_forms:
        aero = StageAero(
            ref_area=math.pi * (sf["diam"] / 2.0) ** 2,
            ref_length=sf["diam"],
            cm_alpha=sf["cm_alpha"],
        )
        stages.append(Stage(
            name=sf["name"], dry_mass=sf["dry"], propellant_mass=sf["prop"],
            isp_sea=sf["isp_sea"], isp_vac=sf["isp_vac"], thrust_vac=sf["thrust_kgf"] * 9.81,
            diameter=sf["diam"], length=sf["length"], gimbal_max_deg=sf["gimbal"], aero=aero,
        ))
    return Vehicle(
        name=preset_name, stages=stages, payload_mass=payload_mass, payload_diameter=payload_diam,
        payload_cd0=payload_cd0, payload_cl_alpha=payload_cl, payload_cm_alpha=payload_cm,
    )


# --------------------------------------------------------------- liftoff check --
vehicle_preview = build_vehicle()
if vehicle_preview.stages:
    s0 = vehicle_preview.stages[0]
    tw = (s0.thrust_vac * (s0.isp_sea / s0.isp_vac)) / (vehicle_preview.total_mass() * 9.81)
    if tw < 1.0:
        st.warning(f"Stage 1 sea-level thrust-to-weight ratio is {tw:.2f} (< 1.0) - this vehicle cannot lift off as configured. Increase thrust or reduce mass.")

# ------------------------------------------------------------------- run --
if run_clicked:
    vehicle = build_vehicle()
    guidance = GuidanceProgram(
        launch_azimuth_deg=azimuth, trajectory_type=traj_type,
        t_vertical=t_vertical, t_pitch=t_pitch, pitch_kick_deg=pitch_kick,
        boost_pitch_deg=boost_pitch,
    )
    sim = Simulation(vehicle, guidance, launch_lat_deg=launch_lat, launch_lon_deg=launch_lon,
                      dt_boost=dt_boost, dt_coast=dt_coast, t_max=t_max)
    with st.spinner("Integrating equations of motion..."):
        result = sim.run()
    st.session_state["result"] = result
    st.session_state["vehicle"] = vehicle

# --------------------------------------------------------------- results --
if "result" not in st.session_state:
    st.info("Configure a vehicle and launch parameters in the sidebar, then click **Run Simulation**.")
    st.stop()

result = st.session_state["result"]
vehicle = st.session_state["vehicle"]
df = result.dataframe

if df.empty or len(df) < 2:
    st.error("Simulation terminated almost immediately (likely could not lift off). Check the thrust-to-weight warning above.")
    st.stop()

metrics = compute_metrics(result, vehicle)

st.markdown("### Key results")
cols = st.columns(5)
cols[0].metric("Apogee", f"{metrics.apogee_km:,.1f} km")
cols[1].metric("Downrange", f"{metrics.total_range_km:,.1f} km")
cols[2].metric("Max Mach", f"{metrics.max_mach:,.2f}")
cols[3].metric("Max-Q", f"{metrics.max_dynamic_pressure_kpa:,.0f} kPa")
cols[4].metric("Flight time", f"{metrics.total_flight_time_s:,.0f} s")

tab_svg, tab_3d, tab_analysis, tab_data = st.tabs(
    ["Animated Flight Report (SVG)", "3D Trajectory", "Analysis Summary", "Raw Data / Export"]
)

with tab_svg:
    st.caption("Hand-built animated SVG - not a plotting-library GIF. Scrub or press Play; every chart shares the same synced timeline.")
    html = build_animated_svg_report(df, title=vehicle.name)
    components.html(html, height=1500, scrolling=True)

with tab_3d:
    try:
        import plotly.graph_objects as go
        r_earth = 6371.0
        x = df["x_eci"].to_numpy() / 1000.0
        y = df["y_eci"].to_numpy() / 1000.0
        z = df["z_eci"].to_numpy() / 1000.0
        fig = go.Figure()
        u, v_ = np.mgrid[0:2 * np.pi:40j, 0:np.pi:24j]
        fig.add_surface(
            x=r_earth * np.cos(u) * np.sin(v_), y=r_earth * np.sin(u) * np.sin(v_), z=r_earth * np.cos(v_),
            colorscale=[[0, "#0b3d63"], [1, "#0b3d63"]], showscale=False, opacity=0.55, name="Earth",
        )
        fig.add_scatter3d(x=x, y=y, z=z, mode="lines", line=dict(color="#ffb454", width=5), name="Trajectory (ECI)")
        fig.update_layout(
            height=720, margin=dict(l=0, r=0, t=30, b=0),
            scene=dict(aspectmode="data", xaxis_title="x (km)", yaxis_title="y (km)", zaxis_title="z (km)"),
            paper_bgcolor="rgba(0,0,0,0)",
        )
        st.plotly_chart(fig, use_container_width=True)
        st.caption("Earth-Centered Inertial frame. WebGL 3D view - bonus visualization alongside the SVG report.")
    except ImportError:
        st.warning("Install `plotly` for the 3D trajectory view: `pip install plotly`")

with tab_analysis:
    st.markdown("#### Flight metrics")
    table = metrics_table(metrics)
    st.table(pd.DataFrame(table, columns=["Metric", "Value"]).set_index("Metric"))

    st.markdown("#### Stage summary")
    stage_rows = []
    for i, s in enumerate(vehicle.stages):
        tw = (s.thrust_vac * (s.isp_sea / s.isp_vac)) / (vehicle.total_mass() * 9.81) if i == 0 else float("nan")
        stage_rows.append({
            "Stage": i + 1, "Name": s.name, "Dry mass (kg)": s.dry_mass, "Propellant (kg)": s.propellant_mass,
            "Isp sea/vac (s)": f"{s.isp_sea:.0f} / {s.isp_vac:.0f}", "Vacuum thrust (kN)": s.thrust_vac / 1000.0,
            "Burn time (s)": f"{s.burn_time:.1f}", "Cm_alpha": s.aero.cm_alpha,
        })
    st.dataframe(pd.DataFrame(stage_rows), use_container_width=True, hide_index=True)

    st.markdown("#### Notes on this model's scope")
    st.markdown(
        "- The aerodynamic model is a simplified Mach/angle-of-attack-dependent drag + linear-in-`sin(alpha)` "
        "lift/moment model, adequate to show realistic-magnitude 6DOF behavior (including tumbling once "
        "uncontrolled) but not a substitute for wind-tunnel/CFD-derived coefficients.\n"
        "- Guidance is open-loop (gravity turn or a fixed commanded pitch), tracked by a torque-saturated "
        "attitude controller abstracting TVC/fin actuation - there is no closed-loop range targeting/impact-point "
        "prediction solver in this version.\n"
        "- Gravity is point-mass (no J2 oblateness term); Earth is treated as a sphere of radius 6371 km."
    )

with tab_data:
    st.markdown(f"**{len(df):,} timesteps.** Termination: {result.termination_reason}.")
    st.dataframe(df, use_container_width=True, height=420)
    c1, c2 = st.columns(2)
    c1.download_button("Download trajectory CSV", dataframe_to_csv_bytes(df), file_name="trajectory.csv", mime="text/csv", use_container_width=True)
    c2.download_button("Download metrics JSON", metrics_to_json_bytes(metrics), file_name="metrics.json", mime="application/json", use_container_width=True)
