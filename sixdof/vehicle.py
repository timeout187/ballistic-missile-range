"""Vehicle definition: staged rocket with mass, propulsion, inertia and
aerodynamic properties needed to drive the 6DOF equations of motion."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List

from .constants import G0


@dataclass
class StageAero:
    """Simplified but Mach/angle-of-attack dependent aerodynamic model.

    Cd = Cd0(Mach) + k_alpha * alpha_total^2      (axial drag build-up with AoA)
    CN = CL_alpha * alpha_total                     (normal force coefficient, linear region)
    Cm = Cm0 + Cm_alpha * alpha_total - Cm_q * (q * ref_length / (2V))   (pitch/yaw moment)

    Cm_alpha < 0  -> statically stable (weathercocks into the relative wind)
    Cm_alpha > 0  -> statically unstable (will tend to tumble once uncontrolled)
    """
    ref_area: float          # m^2, aerodynamic reference area (usually body cross-section)
    ref_length: float        # m, reference length for moment coefficients (usually diameter)
    cd0_subsonic: float = 0.30
    cd0_transonic: float = 0.62   # peak drag near Mach 1
    cd0_supersonic: float = 0.28
    k_alpha: float = 0.4          # additional drag per rad^2 of total angle of attack
    cl_alpha: float = 2.2         # per radian, normal-force-curve slope
    cm0: float = 0.0
    cm_alpha: float = -0.6        # per radian; negative = statically stable
    cm_q: float = 6.0             # pitch/yaw damping coefficient

    def cd0(self, mach: float) -> float:
        if mach < 0.8:
            return self.cd0_subsonic
        if mach < 1.2:
            # smooth peak through the transonic drag rise
            t = (mach - 0.8) / 0.4
            shape = math.sin(math.pi * t)
            return self.cd0_subsonic + (self.cd0_transonic - self.cd0_subsonic) * shape
        # supersonic decay back toward cd0_supersonic
        decay = math.exp(-(mach - 1.2) / 3.0)
        return self.cd0_supersonic + (self.cd0_transonic - self.cd0_supersonic) * decay


@dataclass
class Stage:
    name: str
    dry_mass: float           # kg, structure mass of this stage only (excludes upper stages/payload)
    propellant_mass: float    # kg
    isp_sea: float             # s
    isp_vac: float              # s
    thrust_vac: float          # N, vacuum thrust
    diameter: float           # m
    length: float = 8.0        # m, used only for inertia estimation
    gimbal_max_deg: float = 6.0
    max_control_torque: float = 0.0   # N*m; 0 -> auto-estimated from thrust*lever arm
    aero: StageAero = None
    burn_time: float = 0.0     # s; 0 -> auto-computed from propellant & mass flow

    def __post_init__(self):
        if self.aero is None:
            area = math.pi * (self.diameter / 2.0) ** 2
            self.aero = StageAero(ref_area=area, ref_length=self.diameter)
        mdot = self.mass_flow_rate()
        if self.burn_time <= 0.0 and mdot > 0:
            self.burn_time = self.propellant_mass / mdot
        if self.max_control_torque <= 0.0:
            # crude but physically-scaled default: TVC deflection of a few
            # degrees acting through a lever arm of roughly half the stage length
            lever = 0.35 * self.length
            self.max_control_torque = self.thrust_vac * math.sin(math.radians(self.gimbal_max_deg)) * lever

    def mass_flow_rate(self) -> float:
        return self.thrust_vac / (self.isp_vac * G0)

    def thrust_at_pressure(self, ambient_pressure: float, sea_level_pressure: float = 101325.0) -> float:
        """Interpolate thrust between vacuum and sea-level rating by ambient
        pressure ratio (standard rocket-motor back-pressure approximation)."""
        frac = max(0.0, min(1.0, ambient_pressure / sea_level_pressure))
        thrust_sea = self.thrust_vac * (self.isp_sea / self.isp_vac)
        return self.thrust_vac - frac * (self.thrust_vac - thrust_sea)

    def isp_at_pressure(self, ambient_pressure: float, sea_level_pressure: float = 101325.0) -> float:
        frac = max(0.0, min(1.0, ambient_pressure / sea_level_pressure))
        return self.isp_vac - frac * (self.isp_vac - self.isp_sea)


@dataclass
class Vehicle:
    name: str
    stages: List[Stage] = field(default_factory=list)
    payload_mass: float = 500.0  # kg
    payload_diameter: float = 0.6  # m
    payload_cd0: float = 0.15
    payload_cl_alpha: float = 1.5
    payload_cm_alpha: float = -0.8  # reentry vehicles are usually made aerodynamically stable

    def total_mass(self) -> float:
        return self.payload_mass + sum(s.dry_mass + s.propellant_mass for s in self.stages)

    def inertia_estimate(self, stage_idx: int, current_mass: float) -> "tuple[float, float]":
        """Rough (Ixx, Iyy=Izz) estimate for the current stack, treating it as
        a slender uniform cylinder. Adequate for illustrating rotational
        dynamics/control response magnitudes; replace with vehicle-specific
        inertia data for research-grade fidelity."""
        stage = self.stages[stage_idx]
        radius = stage.diameter / 2.0
        length = max(stage.length, stage.diameter * 3)
        ixx = 0.5 * current_mass * radius ** 2
        iyy = current_mass * (3 * radius ** 2 + length ** 2) / 12.0
        return max(ixx, 1.0), max(iyy, 1.0)
