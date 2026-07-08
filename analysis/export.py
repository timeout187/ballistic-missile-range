"""CSV / JSON export helpers for a simulation run."""

from __future__ import annotations

import json

import pandas as pd

from .metrics import FlightMetrics


def dataframe_to_csv_bytes(df: pd.DataFrame) -> bytes:
    return df.to_csv(index=False).encode("utf-8")


def metrics_to_json_bytes(m: FlightMetrics) -> bytes:
    return json.dumps(m.__dict__, indent=2, default=str).encode("utf-8")


def dataframe_to_json_bytes(df: pd.DataFrame) -> bytes:
    return df.to_json(orient="records").encode("utf-8")
