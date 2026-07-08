"""
Open-loop / attitude-hold guidance program.

Three boost profiles, selectable in the GUI, echoing the trajectory choices
of the original planar tool but now realized as genuine 3D attitude targets
tracked by a torque-limited attitude controller (see dynamics.py):

  - "gravity_turn":  vertical rise -> a brief pitch-over kick -> zero
    commanded angle of attack for the remainder of powered flight (the
    vehicle "prograde-locks", i.e. the guidance simply points the nose
    along the velocity vector and lets gravity rotate the flight-path
    angle down range, exactly the classical gravity-turn technique).

  - "fixed_pitch":   vertical rise -> pitch-over kick -> holds a constant
    commanded pitch angle (from local horizontal) for the rest of powered
    flight. Used for both "minimum energy" style lofted trajectories and
    user-specified "depressed" trajectories; the theoretical minimum-energy
    angle for a *given* target range can be computed with
    `minimum_energy_gamma()` below and entered as the boost pitch.

  - "depressed": identical mechanics to fixed_pitch, provided as a distinct
    label in the UI for clarity when the user is deliberately flying a
    low, depressed-trajectory profile (e.g. SLBM-style).

At and after final-stage burnout, guidance authority is withdrawn entirely
(the controller is simply not called) so the vehicle coasts and re-enters
under aerodynamic + inertial dynamics alone -- free to weathercock if
statically stable, or tumble if not.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


@dataclass
class GuidanceProgram:
    launch_azimuth_deg: float = 90.0     # 0 = North, 90 = East
    trajectory_type: str = "gravity_turn"  # gravity_turn | fixed_pitch | depressed
    t_vertical: float = 6.0              # s of pure vertical rise
    t_pitch: float = 10.0                # s over which the pitch-over kick is applied
    pitch_kick_deg: float = 3.0          # initial tip-over angle
    boost_pitch_deg: float = 45.0        # used by fixed_pitch / depressed after the kick

    def __post_init__(self):
        self._up0 = None
        self._north0 = None
        self._east0 = None
        self._horiz0 = None

    def set_launch_frame(self, r_eci_launch: np.ndarray):
        """Freeze a launch-fixed local ENU-style basis used as the guidance
        reference throughout powered flight (valid for the tens-to-few-
        hundred seconds of boost; Earth's rotation over that span is a
        second-order effect for pointing purposes)."""
        up = r_eci_launch / (np.linalg.norm(r_eci_launch) + 1e-12)
        z_axis = np.array([0.0, 0.0, 1.0])
        east = np.cross(z_axis, up)
        if np.linalg.norm(east) < 1e-9:
            east = np.array([1.0, 0.0, 0.0])
        east = east / np.linalg.norm(east)
        north = np.cross(up, east)
        self._up0 = up
        self._east0 = east
        self._north0 = north
        az = math.radians(self.launch_azimuth_deg)
        self._horiz0 = math.cos(az) * north + math.sin(az) * east

    def target_direction(self, t: float, vel_eci: np.ndarray, powered: bool):
        """Return (active, unit_direction_eci) the guidance wants the nose
        pointed at, or (False, None) if guidance authority is withdrawn."""
        if not powered:
            return False, None

        if t < self.t_vertical:
            return True, self._up0

        if t < self.t_vertical + self.t_pitch:
            frac = (t - self.t_vertical) / max(self.t_pitch, 1e-6)
            kick = math.radians(self.pitch_kick_deg) * frac
            direction = math.cos(kick) * self._up0 + math.sin(kick) * self._horiz0
            return True, direction / np.linalg.norm(direction)

        if self.trajectory_type == "gravity_turn":
            speed = np.linalg.norm(vel_eci)
            if speed < 1.0:
                # not enough velocity yet to define prograde; hold the kick angle
                kick = math.radians(self.pitch_kick_deg)
                direction = math.cos(kick) * self._up0 + math.sin(kick) * self._horiz0
                return True, direction / np.linalg.norm(direction)
            return True, vel_eci / speed

        # fixed_pitch / depressed: hold a constant inertial pitch angle
        gamma = math.radians(self.boost_pitch_deg)
        direction = math.cos(gamma) * self._horiz0 + math.sin(gamma) * self._up0
        return True, direction / np.linalg.norm(direction)


def minimum_energy_gamma(range_angle_rad: float) -> float:
    """Classical minimum-energy burnout flight-path angle for a target
    downrange central angle `phi` (radians), per Wright (1992):

        gamma_burnout = 1/2 * atan( sin(phi) / (cos(phi) - 1) )

    Provided as an analysis/diagnostic aid; the GUI reports it alongside
    the user's chosen boost pitch rather than closing the loop on range
    automatically (closing that loop would require an iterative range
    solver, out of scope for this tool -- see README).
    """
    phi = range_angle_rad
    denom = math.cos(phi) - 1.0
    if abs(denom) < 1e-9:
        return math.radians(45.0)
    return 0.5 * math.atan2(math.sin(phi), denom)
