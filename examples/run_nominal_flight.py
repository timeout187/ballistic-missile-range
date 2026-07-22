"""Run a single preset vehicle headlessly (no Streamlit) and print a
flight-performance summary.

Usage:
    python examples/run_nominal_flight.py [preset-name]

If no preset name is given, defaults to "Russia - Scud-B". Use quotes
around the name since it contains spaces, e.g.:

    python examples/run_nominal_flight.py "DPRK - TD-2"
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from analysis.metrics import compute_metrics, metrics_table
from sixdof.guidance import GuidanceProgram
from sixdof.presets import PRESETS
from sixdof.simulation import Simulation


def main():
    preset_name = sys.argv[1] if len(sys.argv) > 1 else "Russia - Scud-B"
    if preset_name not in PRESETS:
        print(f"Unknown preset {preset_name!r}. Available presets:")
        for name in PRESETS:
            print(f"  {name}")
        sys.exit(1)

    vehicle = PRESETS[preset_name]
    guidance = GuidanceProgram(launch_azimuth_deg=90, trajectory_type="gravity_turn")
    sim = Simulation(vehicle, guidance, launch_lat_deg=35.0, launch_lon_deg=45.0, t_max=3600)

    print(f"Running {preset_name} ...")
    result = sim.run()
    metrics = compute_metrics(result, vehicle)

    for label, value in metrics_table(metrics):
        print(f"{label:32s} {value}")


if __name__ == "__main__":
    main()
