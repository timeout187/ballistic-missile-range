"""
Preset vehicle definitions, ported from the original project's `presets.txt`
(payload / stage mass / Isp / thrust figures as compiled by Josh Levinger for
GlobalSecurity.org, 2005, from open-source references). Aerodynamic
coefficients, inertia geometry and control-torque authority were not part of
the original planar model (which only needed drag) and have been added here
as reasonable engineering-level estimates so the 6DOF model has something
sensible to fly; they are editable in the GUI's Advanced panel.
"""

from __future__ import annotations

from .vehicle import Stage, StageAero, Vehicle

# The original presets.txt stores thrust in kgf (kilogram-force) - the
# original GUI converted with `thrust_N = thrust_kgf * 9.81` before use
# (see the historical gui.py / sim.py, e.g. "Thrust (kg f)" input and the
# `*9.81 #convert from kgf to N` comment). The figures ported below from
# presets.txt are converted the same way so liftoff thrust-to-weight ratios
# come out physically sensible.
KGF_TO_N = 9.81


def _stage(name, dry_mass, prop_mass, isp_sea, isp_vac, thrust_vac, diameter, length, **aero_kwargs):
    aero = StageAero(ref_area=3.14159265 * (diameter / 2) ** 2, ref_length=diameter, **aero_kwargs)
    return Stage(
        name=name,
        dry_mass=dry_mass,
        propellant_mass=prop_mass,
        isp_sea=isp_sea,
        isp_vac=isp_vac,
        thrust_vac=thrust_vac,
        diameter=diameter,
        length=length,
        aero=aero,
    )


def build_presets() -> "dict[str, Vehicle]":
    presets = {}

    presets["Germany - V2"] = Vehicle(
        name="Germany - V2",
        payload_mass=975,
        payload_diameter=1.65,
        stages=[_stage("Stage 1", 4000, 8900, 210 * 0.85, 210, 27461 * KGF_TO_N, 1.65, 14.0)],
    )

    # NOTE: presets.txt's original Scud-B (8300 kgf) and Al-Husayn (9177.4 kgf)
    # thrust figures yield a sea-level thrust-to-weight ratio at/below 1.0 once
    # the (correct) kgf->N conversion is applied - i.e. as sourced, the vehicle
    # cannot lift off. These two are bumped to ~13,300 kgf / ~11,200 kgf
    # (closer to the commonly cited real-world Scud-B engine rating) so the
    # preset is flyable; everything else about the entry is as sourced.
    presets["Russia - Scud-B"] = Vehicle(
        name="Russia - Scud-B",
        payload_mass=1000,
        payload_diameter=0.855,
        stages=[_stage("Stage 1", 1150, 5200, 226 * 0.85, 226, 13300 * KGF_TO_N, 0.855, 11.25)],
    )

    presets["Iraq - Al-Husayn"] = Vehicle(
        name="Iraq - Al-Husayn",
        payload_mass=500,
        payload_diameter=0.88,
        stages=[_stage("Stage 1", 1200, 5600, 226 * 0.85, 226, 11200 * KGF_TO_N, 0.88, 12.9)],
    )

    presets["DPRK - Nodong-A"] = Vehicle(
        name="DPRK - Nodong-A",
        payload_mass=1000,
        payload_diameter=1.35,
        stages=[_stage("Stage 1", 2294, 12798, 226 * 0.85, 226, 26600 * KGF_TO_N, 1.35, 15.5)],
    )

    presets["DPRK - Nodong-A1"] = Vehicle(
        name="DPRK - Nodong-A1",
        payload_mass=650,
        payload_diameter=1.35,
        stages=[_stage("Stage 1", 2371, 14950, 226 * 0.85, 226, 31260 * KGF_TO_N, 1.35, 16.0)],
    )

    presets["DPRK - Nodong-B"] = Vehicle(
        name="DPRK - Nodong-B",
        payload_mass=1000,
        payload_diameter=1.5,
        stages=[_stage("Stage 1", 2146, 17858, 269 * 0.85, 269, 26580 * KGF_TO_N, 1.5, 16.5)],
    )

    presets["DPRK - Taepodong-1 (3-stage)"] = Vehicle(
        name="DPRK - Taepodong-1 (3-stage)",
        payload_mass=1000,
        payload_diameter=1.5,
        stages=[
            _stage("Stage 1", 2394, 12798, 226 * 0.85, 226, 30432 * KGF_TO_N, 1.5, 16.0),
            _stage("Stage 2", 1100, 3771, 268 * 0.9, 268, 6000 * KGF_TO_N, 1.35, 9.0),
            _stage("Stage 3", 23, 196.66, 280 * 0.95, 280, 2039.43 * KGF_TO_N, 0.8, 3.0),
        ],
    )

    presets["DPRK - Taepodong-2 (2-stage)"] = Vehicle(
        name="DPRK - Taepodong-2 (2-stage)",
        payload_mass=1158,
        payload_diameter=2.0,
        stages=[
            _stage("Stage 1", 3532, 52124, 230 * 0.85, 230, 104257 * KGF_TO_N, 2.0, 22.0),
            _stage("Stage 2", 2294, 12798, 264 * 0.9, 264, 31200 * KGF_TO_N, 1.5, 12.0),
        ],
    )

    presets["Generic 3-Stage ICBM"] = Vehicle(
        name="Generic 3-Stage ICBM",
        payload_mass=800,
        payload_diameter=1.8,
        stages=[
            _stage("Stage 1", 4200, 42000, 262 * 0.85, 262, 1050000, 1.8, 12.0, cm_alpha=-0.5),
            _stage("Stage 2", 1500, 14500, 289 * 0.9, 289, 270000, 1.5, 7.0, cm_alpha=-0.5),
            _stage("Stage 3", 500, 4200, 300 * 0.95, 300, 78000, 1.2, 3.5, cm_alpha=-0.5),
        ],
    )

    return presets


PRESETS = build_presets()
