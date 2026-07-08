"""
Preset vehicle definitions, copied verbatim from the original project's
`legacy_2005_planar_tool/presets.txt` (payload / stage mass / Isp / thrust /
diameter figures as compiled by Josh Levinger for GlobalSecurity.org, 2005,
from open-source references).

The ballistic numbers below are NOT modified from the source sheet:

  * propellant_mass   <- fuelmass[i]
  * dry_mass          <- drymass[i]
  * isp_sea = isp_vac <- Isp0[i]   (the sheet gives a single Isp per stage;
                                     it is used at all altitudes, unchanged)
  * thrust_vac        <- thrust0[i] * 9.81   (the sheet's thrust is in kgf
                                     (kilogram-force); the original GUI
                                     converted to Newtons with * 9.81 before
                                     use, and so do we)
  * diameter          <- missilediam
  * payload_mass      <- payload
  * payload_diameter  <- rvdiam if the sheet gives one, else missilediam

The only values NOT present in the source sheet are per-stage LENGTH and the
aerodynamic coefficients. The original tool was a 2D point-mass model and
never needed them; the 6DOF model does (length feeds the rotational-inertia
estimate, the aero coefficients feed the attitude dynamics). Those are
documented engineering estimates, editable in the GUI's Advanced panel - they
do not alter the ballistic data above.
"""

from __future__ import annotations

import math

from .vehicle import Stage, StageAero, Vehicle

# The sheet's thrust is in kgf (kilogram-force); 1 kgf = 9.81 N. The original
# gui.py converted with `* 9.81 #convert from kgf to N` before use.
KGF_TO_N = 9.81


def _stage(name, drymass, fuelmass, isp0, thrust0_kgf, diameter, length):
    """Build a Stage from the sheet's per-stage fields. drymass, fuelmass,
    isp0 and thrust0_kgf are taken exactly from presets.txt; a single Isp is
    used for both sea level and vacuum (the sheet only lists one). length is a
    geometry estimate the 2D sheet did not carry (see module docstring)."""
    aero = StageAero(ref_area=math.pi * (diameter / 2) ** 2, ref_length=diameter)
    return Stage(
        name=name,
        dry_mass=drymass,
        propellant_mass=fuelmass,
        isp_sea=isp0,
        isp_vac=isp0,
        thrust_vac=thrust0_kgf * KGF_TO_N,
        diameter=diameter,
        length=length,
        aero=aero,
    )


def build_presets() -> "dict[str, Vehicle]":
    presets = {}

    # 'Germany - V2': payload 975, missilediam 1.65, rvdiam 1.65,
    #   fuelmass [0,8900], drymass [0,4000], Isp0 [0,210], thrust0 [0,27461]
    presets["Germany - V2"] = Vehicle(
        name="Germany - V2",
        payload_mass=975,
        payload_diameter=1.65,
        stages=[_stage("Stage 1", 4000, 8900, 210, 27461, 1.65, 14.0)],
    )

    # 'Russia - Scud-B': payload 1000, missilediam .855, rvdiam 0,
    #   fuelmass [0,5200], drymass [0,1150], Isp0 [0,226], thrust0 [0,8300]
    presets["Russia - Scud-B"] = Vehicle(
        name="Russia - Scud-B",
        payload_mass=1000,
        payload_diameter=0.855,
        stages=[_stage("Stage 1", 1150, 5200, 226, 8300, 0.855, 11.25)],
    )

    # 'Iraq - Al-Husayn': payload 500, missilediam 0.88, rvdiam 0,
    #   fuelmass [0,5600], drymass [0,1200], Isp0 [0,226], thrust0 [0,9177.4]
    presets["Iraq - Al-Husayn"] = Vehicle(
        name="Iraq - Al-Husayn",
        payload_mass=500,
        payload_diameter=0.88,
        stages=[_stage("Stage 1", 1200, 5600, 226, 9177.4, 0.88, 12.9)],
    )

    # 'DPRK - Nodong-A': payload 1000, missilediam 1.35, rvdiam 0,
    #   fuelmass [0,12798], drymass [0,2294], Isp0 [0,226], thrust0 [0,26600]
    presets["DPRK - Nodong-A"] = Vehicle(
        name="DPRK - Nodong-A",
        payload_mass=1000,
        payload_diameter=1.35,
        stages=[_stage("Stage 1", 2294, 12798, 226, 26600, 1.35, 15.5)],
    )

    # 'DPRK - Nodong-A1': payload 650, missilediam 1.35, rvdiam 0,
    #   fuelmass [0,14950], drymass [0,2371], Isp0 [0,226], thrust0 [0,31260]
    presets["DPRK - Nodong-A1"] = Vehicle(
        name="DPRK - Nodong-A1",
        payload_mass=650,
        payload_diameter=1.35,
        stages=[_stage("Stage 1", 2371, 14950, 226, 31260, 1.35, 16.0)],
    )

    # 'DPRK - Nodong-B': payload 1000, missilediam 1.5, rvdiam 0,
    #   fuelmass [0,17858], drymass [0,2146], Isp0 [0,269], thrust0 [0,26580]
    presets["DPRK - Nodong-B"] = Vehicle(
        name="DPRK - Nodong-B",
        payload_mass=1000,
        payload_diameter=1.5,
        stages=[_stage("Stage 1", 2146, 17858, 269, 26580, 1.5, 16.5)],
    )

    # 'DPRK - TD-1': payload 1000, missilediam 1.5, rvdiam 0, numstages 3,
    #   fuelmass [0,12798,3771,196.66], drymass [0,2394,1100,23],
    #   Isp0 [0,226,268,280], thrust0 [0,30432,6000,2039.43]
    presets["DPRK - TD-1"] = Vehicle(
        name="DPRK - TD-1",
        payload_mass=1000,
        payload_diameter=1.5,
        stages=[
            _stage("Stage 1", 2394, 12798, 226, 30432, 1.5, 16.0),
            _stage("Stage 2", 1100, 3771, 268, 6000, 1.5, 9.0),
            _stage("Stage 3", 23, 196.66, 280, 2039.43, 1.5, 3.0),
        ],
    )

    # 'DPRK - TD-2': payload 1158, missilediam 2.0, rvdiam 0, numstages 2,
    #   fuelmass [0,52124,12798], drymass [0,3532,2294],
    #   Isp0 [0,230,264], thrust0 [0,104257,31200]
    presets["DPRK - TD-2"] = Vehicle(
        name="DPRK - TD-2",
        payload_mass=1158,
        payload_diameter=2.0,
        stages=[
            _stage("Stage 1", 3532, 52124, 230, 104257, 2.0, 22.0),
            _stage("Stage 2", 2294, 12798, 264, 31200, 2.0, 12.0),
        ],
    )

    return presets


PRESETS = build_presets()
