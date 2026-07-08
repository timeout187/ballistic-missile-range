# Legacy: original 2005 planar range/altitude tool

This directory preserves the original project this repository was forked
from, kept for reference and attribution:

Simulates the flight of intercontinental ballistic missiles based on launch
parameters. Written by Josh Levinger for GlobalSecurity.org in June 2005.
Original credit for the simulation goes to Dr. David Wright at MIT, who
wrote a version in BASIC for his paper "Depressed Trajectory SLBMs",
*Science and Global Security*, 1992, Vol 3, p101-159. GUI updates and
packaging improvements by Karsten Wolf (@karstenw).

It requires **Python 2.7 + wxPython** (end-of-life, effectively unrunnable
on a modern system without a legacy environment) and models the flight as a
**2D point mass in a single vertical plane** - range and altitude only, no
attitude/rotation, no lateral motion, no wind.

The active project has moved to the `sixdof/` package and `app.py` at the
repository root: a full six-degree-of-freedom (3D translation + rotation)
rigid-body simulator with a modern Streamlit GUI. See the top-level
`README.md` for details. `presets.txt` here was the data source the new
`sixdof/presets.py` was ported from (with unit corrections - see that
file's comments).
