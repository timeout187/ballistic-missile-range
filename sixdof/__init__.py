"""
sixdof - a six-degrees-of-freedom (6DOF) rigid-body flight dynamics engine
for ballistic/guided missile and rocket trajectories.

Successor to the original 2005 planar (range/altitude only) simulation by
Josh Levinger / Dr. David Wright. This package replaces the 2D point-mass
model with a full 3D translational + rotational rigid-body simulation:

    - 3D translational motion integrated in an Earth-Centered Inertial (ECI)
      frame under point-mass gravity.
    - Rotational motion (attitude) integrated with quaternions under a
      rigid-body inertia tensor, driven by aerodynamic moments and a
      saturating attitude-control torque (an abstraction of TVC/fin
      actuation) during powered flight.
    - A 1976 U.S. Standard Atmosphere model and a simplified but
      Mach/angle-of-attack dependent aerodynamic model (drag, lift,
      restoring/damping moment).
    - Multi-stage boost, coast and ballistic re-entry, with the vehicle
      free to tumble or weathercock aerodynamically once control authority
      is removed at final-stage burnout - something the original planar
      model could not represent at all.

Units are SI throughout (meters, kilograms, seconds, radians) unless
otherwise noted in a function's docstring.
"""

from .constants import *  # noqa: F401,F403
