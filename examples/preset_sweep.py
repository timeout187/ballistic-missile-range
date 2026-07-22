"""Run every preset vehicle back-to-back and print a comparison table of
apogee, downrange, max Mach and max axial load - useful for a quick sanity
check after changing the physics core, or as a starting point for a
parameter-sweep script of your own.

Usage:
    python examples/preset_sweep.py
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from analysis.metrics import compute_metrics
from sixdof.guidance import GuidanceProgram
from sixdof.presets import PRESETS
from sixdof.simulation import Simulation


def main():
    header = f"{'Vehicle':22s} {'Apogee (km)':>12s} {'Range (km)':>12s} {'Max Mach':>9s} {'Max g':>7s}"
    print(header)
    print("-" * len(header))

    for name, vehicle in PRESETS.items():
        guidance = GuidanceProgram(launch_azimuth_deg=90, trajectory_type="gravity_turn")
        sim = Simulation(vehicle, guidance, launch_lat_deg=35.0, launch_lon_deg=45.0, t_max=3600)
        result = sim.run()
        m = compute_metrics(result, vehicle)
        outcome = "" if result.impacted else "  (did not impact)"
        print(f"{name:22s} {m.apogee_km:12.1f} {m.total_range_km:12.1f} {m.max_mach:9.2f} {m.max_accel_g:7.1f}{outcome}")


if __name__ == "__main__":
    main()
