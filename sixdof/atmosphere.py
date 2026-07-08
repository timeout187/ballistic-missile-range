"""
1976 U.S. Standard Atmosphere (simplified, geopotential layers to 86 km).

Returns density, pressure, temperature and local speed of sound for a
given geometric altitude above sea level. Above 86 km the model blends
into an exponential vacuum-tending tail (adequate for reentry/apogee
bookkeeping; not a substitute for a thermospheric model).
"""

from __future__ import annotations

import math

from .constants import R_AIR, GAMMA_AIR

# Layer base geopotential altitude (m), base temperature (K), lapse rate (K/m)
_LAYERS = [
    (0.0,     288.15, -0.0065),
    (11000.0, 216.65,  0.0),
    (20000.0, 216.65,  0.001),
    (32000.0, 228.65,  0.0028),
    (47000.0, 270.65,  0.0),
    (51000.0, 270.65, -0.0028),
    (71000.0, 214.65, -0.002),
    (84852.0, 186.946, 0.0),
]

_P0 = 101325.0     # Pa, sea level standard pressure
_G0 = 9.80665
_R_EARTH_GEOPOT = 6356766.0  # m, radius used for geometric->geopotential conversion


def _geopotential_height(h_geometric: float) -> float:
    return _R_EARTH_GEOPOT * h_geometric / (_R_EARTH_GEOPOT + h_geometric)


def atmosphere(h_geometric: float):
    """Return (rho [kg/m^3], pressure [Pa], temperature [K], speed_of_sound [m/s])."""
    if h_geometric < 0:
        h_geometric = 0.0

    if h_geometric > 86000.0:
        # Thin exponential tail so drag/lift smoothly vanish rather than
        # discontinuously cutting off. Not physically exact above 86 km,
        # but forces here are already negligible for trajectory purposes.
        rho_86, p_86, t_86, _ = atmosphere(86000.0)
        scale_height = 7000.0
        rho = rho_86 * math.exp(-(h_geometric - 86000.0) / scale_height)
        pressure = p_86 * math.exp(-(h_geometric - 86000.0) / scale_height)
        temperature = t_86
        a = math.sqrt(GAMMA_AIR * R_AIR * temperature)
        return max(rho, 1e-14), max(pressure, 1e-9), temperature, a

    h = _geopotential_height(h_geometric)

    pressure = _P0
    base_h, base_t, lapse = _LAYERS[0]
    temperature = base_t

    for i in range(len(_LAYERS)):
        layer_h, layer_t0, layer_lapse = _LAYERS[i]
        next_h = _LAYERS[i + 1][0] if i + 1 < len(_LAYERS) else 1.0e9

        if h <= next_h or i == len(_LAYERS) - 1:
            dh = h - layer_h
            if abs(layer_lapse) < 1e-12:
                temperature = layer_t0
                pressure = pressure * math.exp(-_G0 * dh / (R_AIR * layer_t0))
            else:
                temperature = layer_t0 + layer_lapse * dh
                pressure = pressure * (layer_t0 / temperature) ** (_G0 / (R_AIR * layer_lapse))
            break
        else:
            dh = next_h - layer_h
            if abs(layer_lapse) < 1e-12:
                pressure = pressure * math.exp(-_G0 * dh / (R_AIR * layer_t0))
            else:
                t_next = layer_t0 + layer_lapse * dh
                pressure = pressure * (layer_t0 / t_next) ** (_G0 / (R_AIR * layer_lapse))

    rho = pressure / (R_AIR * temperature)
    a = math.sqrt(GAMMA_AIR * R_AIR * temperature)
    return max(rho, 1e-14), max(pressure, 1e-9), temperature, a
