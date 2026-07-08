"""
Builds a single self-contained HTML document containing hand-built, animated
SVG charts of a flight: the range/altitude profile with a moving vehicle
marker, a ground track, and a strip of synced time-series mini-charts
(altitude, speed/Mach, dynamic pressure, angle of attack, axial load).

Deliberately dependency-free (no plotting library) so every element is a
plain SVG <path>/<circle>/<line>, and the "animation" is a small vanilla-JS
scrubber/player driving element attributes directly -- i.e. a real animated
SVG report, not a video/gif embed.
"""

from __future__ import annotations

import json
import math

import numpy as np
import pandas as pd


def _downsample(df: pd.DataFrame, max_points: int = 600) -> pd.DataFrame:
    if len(df) <= max_points:
        return df.reset_index(drop=True)
    idx = np.linspace(0, len(df) - 1, max_points).astype(int)
    return df.iloc[idx].reset_index(drop=True)


def _series_to_path(xs, ys, x_lo, x_hi, y_lo, y_hi, w, h, pad=8, invert_y=True):
    def sx(x):
        if x_hi - x_lo < 1e-9:
            return pad
        return pad + (x - x_lo) / (x_hi - x_lo) * (w - 2 * pad)

    def sy(y):
        if y_hi - y_lo < 1e-9:
            v = pad
        else:
            v = (y - y_lo) / (y_hi - y_lo) * (h - 2 * pad)
        return (h - pad - v) if invert_y else (pad + v)

    pts = [f"{sx(x):.2f},{sy(y):.2f}" for x, y in zip(xs, ys)]
    return "M" + " L".join(pts), sx, sy


def _pad_range(lo, hi, frac=0.08):
    span = hi - lo
    if span < 1e-9:
        span = max(abs(hi), 1.0)
    return lo - span * frac, hi + span * frac


_MINI_SPECS = [
    ("altitude_km", "Altitude", "km", "#5ec8f8"),
    ("mach", "Mach number", "", "#ffb454"),
    ("dynamic_pressure_Pa_kpa", "Dynamic pressure", "kPa", "#ff6b6b"),
    ("alpha_total_deg", "Total angle of attack", "deg", "#a78bfa"),
    ("accel_g", "Axial load", "g", "#4ade80"),
]


def build_animated_svg_report(df: pd.DataFrame, title: str = "Flight", max_points: int = 600) -> str:
    d = _downsample(df, max_points).copy()
    d["dynamic_pressure_Pa_kpa"] = d["dynamic_pressure_Pa"] / 1000.0

    n = len(d)
    t = d["time"].to_numpy()
    alt = d["altitude_km"].to_numpy()
    rng = d["downrange_km"].to_numpy()
    lat = d["lat_deg"].to_numpy()
    lon = d["lon_deg"].to_numpy()
    pitch = d["pitch_deg"].to_numpy()

    MAIN_W, MAIN_H = 900, 340
    rng_lo, rng_hi = _pad_range(float(rng.min()), float(rng.max()))
    alt_lo, alt_hi = _pad_range(0.0, float(alt.max()))
    main_path, main_sx, main_sy = _series_to_path(rng, alt, rng_lo, rng_hi, alt_lo, alt_hi, MAIN_W, MAIN_H)
    main_x = [round(main_sx(v), 2) for v in rng]
    main_y = [round(main_sy(v), 2) for v in alt]

    GT_W, GT_H = 900, 260
    lon_lo, lon_hi = _pad_range(float(lon.min()), float(lon.max()))
    lat_lo, lat_hi = _pad_range(float(lat.min()), float(lat.max()))
    gt_path, gt_sx, gt_sy = _series_to_path(lon, lat, lon_lo, lon_hi, lat_lo, lat_hi, GT_W, GT_H, invert_y=True)
    gt_x = [round(gt_sx(v), 2) for v in lon]
    gt_y = [round(gt_sy(v), 2) for v in lat]

    MINI_W, MINI_H = 420, 140
    minis = []
    for key, label, unit, color in _MINI_SPECS:
        ys = d[key].to_numpy()
        y_lo, y_hi = _pad_range(float(np.nanmin(ys)), float(np.nanmax(ys)))
        path, msx, msy = _series_to_path(t, ys, float(t.min()), float(t.max()), y_lo, y_hi, MINI_W, MINI_H)
        xs = [round(msx(v), 2) for v in t]
        ys_px = [round(msy(v), 2) for v in ys]
        minis.append({
            "key": key, "label": label, "unit": unit, "color": color, "path": path,
            "xs": xs, "ys": ys_px, "values": [round(float(v), 2) for v in ys],
        })

    data = {
        "n": n,
        "t": [round(float(v), 3) for v in t],
        "mainX": main_x, "mainY": main_y,
        "gtX": gt_x, "gtY": gt_y,
        "pitch": [round(float(v), 2) for v in pitch],
        "altKm": [round(float(v), 3) for v in alt],
        "rngKm": [round(float(v), 3) for v in rng],
        "machV": [round(float(v), 3) for v in d["mach"].to_numpy()],
        "speedV": [round(float(v), 1) for v in d["speed_relative_ms"].to_numpy()],
        "stage": [int(v) for v in d["stage"].to_numpy()],
        "powered": [bool(v) for v in d["powered"].to_numpy()],
    }
    data_json = json.dumps(data)
    minis_json = json.dumps(minis)

    mini_svgs = []
    for i, mi in enumerate(minis):
        mini_svgs.append(f"""
        <div class="mini-card">
          <div class="mini-head">
            <span class="mini-label">{mi['label']}</span>
            <span class="mini-value" id="mini-val-{i}">-- {mi['unit']}</span>
          </div>
          <svg viewBox="0 0 {MINI_W} {MINI_H}" class="mini-svg" preserveAspectRatio="none">
            <path d="{mi['path']}" class="mini-line" style="stroke:{mi['color']}" />
            <line id="mini-cursor-{i}" x1="0" y1="0" x2="0" y2="{MINI_H}" class="cursor-line" />
            <circle id="mini-dot-{i}" r="4" class="mini-dot" style="fill:{mi['color']}" />
          </svg>
        </div>""")
    mini_svgs_html = "\n".join(mini_svgs)

    html = f"""
<div class="sixdof-report">
  <style>
    .sixdof-report {{
      --bg: #0b1220; --panel: #121b2e; --grid: #22314d; --text: #e6edf7;
      --muted: #93a4c3; --accent: #5ec8f8; --line: #7fd0ff;
      font-family: -apple-system, Segoe UI, Roboto, Helvetica, Arial, sans-serif;
      color: var(--text); background: transparent; padding: 4px 2px 12px 2px;
    }}
    @media (prefers-color-scheme: light) {{
      .sixdof-report {{ --bg:#ffffff; --panel:#f4f7fc; --grid:#dde5f2; --text:#101827; --muted:#5b6b85; --accent:#0b76c4; --line:#0b76c4; }}
    }}
    .sixdof-report * {{ box-sizing: border-box; }}
    .sd-title {{ font-size: 15px; font-weight: 600; letter-spacing: .02em; margin: 2px 0 10px 2px; color: var(--text); }}
    .sd-controls {{
      display: flex; align-items: center; gap: 10px; flex-wrap: wrap;
      background: var(--panel); border: 1px solid var(--grid); border-radius: 10px;
      padding: 10px 14px; margin-bottom: 14px;
    }}
    .sd-btn {{
      background: var(--accent); color: #05131f; border: none; border-radius: 7px;
      padding: 7px 14px; font-weight: 600; font-size: 13px; cursor: pointer;
    }}
    .sd-btn:hover {{ filter: brightness(1.08); }}
    .sd-range {{ flex: 1 1 220px; accent-color: var(--accent); }}
    .sd-speed {{ background: var(--bg); color: var(--text); border: 1px solid var(--grid); border-radius: 6px; padding: 5px 8px; font-size: 12px; }}
    .sd-readout {{ font-size: 12.5px; color: var(--muted); min-width: 210px; }}
    .sd-readout b {{ color: var(--text); }}
    .sd-panel {{ background: var(--panel); border: 1px solid var(--grid); border-radius: 10px; padding: 10px 12px 6px 12px; margin-bottom: 14px; }}
    .sd-panel-title {{ font-size: 12.5px; color: var(--muted); margin-bottom: 4px; text-transform: uppercase; letter-spacing: .06em; }}
    .sd-svg {{ width: 100%; height: auto; display: block; }}
    .flight-path {{ fill: none; stroke: var(--line); stroke-width: 2.4; }}
    .flight-path-trace {{ fill: none; stroke: var(--accent); stroke-width: 3.4; stroke-linecap: round; }}
    .gt-path {{ fill: none; stroke: var(--line); stroke-width: 2; stroke-dasharray: 3 3; opacity: .8; }}
    .vehicle-marker {{ fill: #ffd166; stroke: #7a5200; stroke-width: 1; }}
    .grid-line {{ stroke: var(--grid); stroke-width: 1; }}
    .axis-label {{ fill: var(--muted); font-size: 10px; }}
    .mini-grid {{ display: grid; grid-template-columns: repeat(auto-fit, minmax(260px, 1fr)); gap: 12px; }}
    .mini-card {{ background: var(--panel); border: 1px solid var(--grid); border-radius: 10px; padding: 8px 10px; }}
    .mini-head {{ display: flex; justify-content: space-between; font-size: 11.5px; color: var(--muted); margin-bottom: 2px; }}
    .mini-value {{ color: var(--text); font-weight: 600; }}
    .mini-svg {{ width: 100%; height: 90px; }}
    .mini-line {{ fill: none; stroke-width: 2; }}
    .cursor-line {{ stroke: var(--muted); stroke-width: 1; stroke-dasharray: 2 2; }}
    .mini-dot {{ }}
    .sd-stage-badge {{ font-size: 11px; padding: 2px 8px; border-radius: 999px; background: var(--bg); border: 1px solid var(--grid); color: var(--muted); }}
  </style>

  <div class="sd-title">{title} - animated flight report</div>

  <div class="sd-controls">
    <button class="sd-btn" id="sd-play">Play</button>
    <input type="range" id="sd-scrub" class="sd-range" min="0" max="{n - 1}" value="0" step="1" />
    <select id="sd-speed" class="sd-speed">
      <option value="0.25">0.25x</option>
      <option value="0.5">0.5x</option>
      <option value="1" selected>1x</option>
      <option value="4">4x</option>
      <option value="20">20x</option>
      <option value="60">60x</option>
    </select>
    <span class="sd-readout">
      t = <b id="sd-t">0.0</b> s &nbsp;|&nbsp; alt <b id="sd-alt">0.0</b> km &nbsp;|&nbsp;
      range <b id="sd-rng">0.0</b> km &nbsp;|&nbsp; Mach <b id="sd-mach">0.0</b>
      &nbsp;<span class="sd-stage-badge" id="sd-stage">stage 1</span>
    </span>
  </div>

  <div class="sd-panel">
    <div class="sd-panel-title">Altitude vs. Downrange</div>
    <svg viewBox="0 0 {MAIN_W} {MAIN_H}" class="sd-svg" preserveAspectRatio="none" id="sd-main-svg">
      <path d="{main_path}" class="flight-path" />
      <path d="" class="flight-path-trace" id="sd-main-trace" />
      <g id="sd-marker" transform="translate(0,0)">
        <polygon points="-6,5 6,5 0,-11" class="vehicle-marker" />
      </g>
    </svg>
  </div>

  <div class="sd-panel">
    <div class="sd-panel-title">Ground Track (lat / lon)</div>
    <svg viewBox="0 0 {GT_W} {GT_H}" class="sd-svg" preserveAspectRatio="none" id="sd-gt-svg">
      <path d="{gt_path}" class="gt-path" />
      <path d="" class="flight-path-trace" id="sd-gt-trace" />
      <circle id="sd-gt-marker" r="5" class="vehicle-marker" />
    </svg>
  </div>

  <div class="mini-grid">
    {mini_svgs_html}
  </div>
</div>

<script>
(function() {{
  const DATA = {data_json};
  const MINIS = {minis_json};

  function q(id) {{ return document.getElementById(id); }}

  const n = DATA.n;
  const playBtn = q('sd-play');
  const scrub = q('sd-scrub');
  const speedSel = q('sd-speed');
  const marker = q('sd-marker');
  const mainTrace = q('sd-main-trace');
  const gtTrace = q('sd-gt-trace');
  const gtMarker = q('sd-gt-marker');

  let playing = false;
  let idx = 0;
  let lastTs = null;
  let virtualTime = DATA.t[0];

  function tracePath(xs, ys, upto) {{
    let s = "";
    for (let i = 0; i <= upto; i++) {{
      s += (i === 0 ? "M" : " L") + xs[i].toFixed(2) + "," + ys[i].toFixed(2);
    }}
    return s;
  }}

  function render(i) {{
    i = Math.max(0, Math.min(n - 1, i));
    idx = i;
    scrub.value = i;

    marker.setAttribute('transform', `translate(${{DATA.mainX[i]}},${{DATA.mainY[i]}}) rotate(${{-DATA.pitch[i] + 90}})`);
    mainTrace.setAttribute('d', tracePath(DATA.mainX, DATA.mainY, i));

    gtMarker.setAttribute('cx', DATA.gtX[i]);
    gtMarker.setAttribute('cy', DATA.gtY[i]);
    gtTrace.setAttribute('d', tracePath(DATA.gtX, DATA.gtY, i));

    q('sd-t').textContent = DATA.t[i].toFixed(1);
    q('sd-alt').textContent = DATA.altKm[i].toFixed(2);
    q('sd-rng').textContent = DATA.rngKm[i].toFixed(2);
    q('sd-mach').textContent = DATA.machV[i].toFixed(2);
    q('sd-stage').textContent = (DATA.powered[i] ? 'powered - stage ' : 'coast/reentry - stage ') + DATA.stage[i];

    MINIS.forEach((mi, mIdx) => {{
      const cursor = q('mini-cursor-' + mIdx);
      const dot = q('mini-dot-' + mIdx);
      const val = q('mini-val-' + mIdx);
      const x = mi.xs[i], y = mi.ys[i];
      cursor.setAttribute('x1', x); cursor.setAttribute('x2', x);
      dot.setAttribute('cx', x); dot.setAttribute('cy', y);
      val.textContent = mi.values[i].toFixed(2) + ' ' + mi.unit;
    }});
  }}

  function step(ts) {{
    if (!playing) {{ lastTs = null; return; }}
    if (lastTs === null) lastTs = ts;
    const dt = (ts - lastTs) / 1000;
    lastTs = ts;
    const speed = parseFloat(speedSel.value);
    virtualTime += dt * speed;

    let i = idx;
    while (i < n - 1 && DATA.t[i + 1] <= virtualTime) i++;
    if (i >= n - 1) {{
      playing = false;
      playBtn.textContent = 'Play';
      render(n - 1);
      return;
    }}
    render(i);
    requestAnimationFrame(step);
  }}

  playBtn.addEventListener('click', function() {{
    playing = !playing;
    playBtn.textContent = playing ? 'Pause' : 'Play';
    if (playing) {{
      if (idx >= n - 1) {{ idx = 0; virtualTime = DATA.t[0]; }}
      else {{ virtualTime = DATA.t[idx]; }}
      lastTs = null;
      requestAnimationFrame(step);
    }}
  }});

  scrub.addEventListener('input', function() {{
    playing = false;
    playBtn.textContent = 'Play';
    const i = parseInt(scrub.value, 10);
    virtualTime = DATA.t[i];
    render(i);
  }});

  render(0);
}})();
</script>
"""
    return html
