"""Self-contained HTML run report from run_metrics.json and timeseries.csv.

Pure Python (standard library only), so the evaluator writes ``report.html``
at the end of every run inside the container and ``scripts/make_report.py``
can rebuild it on a host without ROS. The page has no external resources:
charts are inline SVG and it opens from disk in any browser.

Contents: the result and provenance, a top view of the travelled track with
the course (targets with tolerance circles and heading arrows, gates and
obstacles), per-target results for setpoint courses, and time plots of the
distance and heading error to the active target, speed and thrust.

Everything shown is copied or plotted from the two input files; the report
computes no new scores. Scoring uses simulation ground truth; thrust is the
guard's commanded force, not measured electrical power.
"""

import csv
import html
import json
import math
from pathlib import Path

PALETTE = ("#2563eb", "#d97706", "#059669", "#db2777", "#7c3aed", "#0891b2", "#65a30d", "#dc2626")
OUTCOME_COLORS = {"reached": "#059669", "advanced": "#2563eb", "superseded": "#2563eb",
                  "timeout": "#dc2626", "unfinished": "#dc2626"}
GATE_COLORS = {"red": "#dc2626", "green": "#16a34a"}

STYLE = """
:root { --bg: #ffffff; --fg: #1f2933; --muted: #616e7c; --grid: #e4e7eb; --card: #f5f7fa;
        --ok: #059669; --bad: #dc2626; --track: #1f2933; }
@media (prefers-color-scheme: dark) {
  :root { --bg: #111418; --fg: #e4e7eb; --muted: #9aa5b1; --grid: #2d333b; --card: #1a1f26;
          --ok: #34d399; --bad: #f87171; --track: #e4e7eb; }
}
* { box-sizing: border-box; }
body { margin: 0; padding: 24px 16px 48px; background: var(--bg); color: var(--fg);
       font: 15px/1.5 system-ui, -apple-system, "Segoe UI", sans-serif; }
main { max-width: 1100px; margin: 0 auto; }
h1 { font-size: 1.6rem; margin: 0 0 4px; }
h2 { font-size: 1.15rem; margin: 32px 0 8px; }
p.sub { color: var(--muted); margin: 0 0 16px; }
.badge { display: inline-block; padding: 2px 10px; border-radius: 999px; font-weight: 600; color: #fff; }
.kpis { display: grid; grid-template-columns: repeat(auto-fit, minmax(150px, 1fr)); gap: 12px; }
.kpi { background: var(--card); border-radius: 8px; padding: 10px 14px; }
.kpi .label { color: var(--muted); font-size: 0.8rem; }
.kpi .value { font-size: 1.25rem; font-weight: 600; font-variant-numeric: tabular-nums; }
.scroll { overflow-x: auto; }
table { border-collapse: collapse; width: 100%; font-variant-numeric: tabular-nums; font-size: 0.9rem; }
th, td { text-align: right; padding: 6px 8px; border-bottom: 1px solid var(--grid); white-space: nowrap; }
th:nth-child(-n+3), td:nth-child(-n+3) { text-align: left; }
th { color: var(--muted); font-weight: 600; }
svg { display: block; width: 100%; height: auto; }
svg text { fill: var(--muted); font: 12px system-ui, sans-serif; }
.axis { stroke: var(--grid); }
.note { color: var(--muted); font-size: 0.85rem; }
dl { display: grid; grid-template-columns: max-content 1fr; gap: 2px 16px; font-size: 0.85rem; }
dt { color: var(--muted); } dd { margin: 0; word-break: break-all; }
"""


def _fmt(value, digits=2, unit=""):
    if value is None or (isinstance(value, float) and not math.isfinite(value)):
        return "–"
    if isinstance(value, bool):
        return "yes" if value else "no"
    if isinstance(value, (int, float)):
        return f"{value:.{digits}f}{unit}"
    return html.escape(str(value))


def _nice_ticks(low, high, count=5):
    """Round tick values spanning [low, high]."""
    if not (math.isfinite(low) and math.isfinite(high)) or high <= low:
        return [low]
    step = 10 ** math.floor(math.log10((high - low) / count))
    for factor in (1, 2, 5, 10):
        if (high - low) / (step * factor) <= count:
            step *= factor
            break
    first = math.ceil(low / step) * step
    return [first + i * step for i in range(int((high - first) / step + 1e-9) + 1)]


def _tick_label(value):
    return f"{value:.0f}" if abs(value) >= 10 or value == int(value) else f"{value:.1f}"


def line_chart(series, x_label, y_label, vlines=(), height=220, width=1000):
    """SVG line chart. ``series``: (name, xs, ys, color); None in ys breaks the line."""
    points = [(x, y) for _, xs, ys, _ in series for x, y in zip(xs, ys) if y is not None]
    if not points:
        return '<p class="note">No data.</p>'
    left, right, top, bottom = 56, 16, 12, 36
    x_low, x_high = min(p[0] for p in points), max(p[0] for p in points)
    y_low, y_high = min(p[1] for p in points), max(p[1] for p in points)
    if x_high == x_low:
        x_high = x_low + 1.0
    pad = (y_high - y_low) * 0.05 or 1.0
    y_low, y_high = y_low - pad, y_high + pad
    sx = lambda x: left + (x - x_low) / (x_high - x_low) * (width - left - right)
    sy = lambda y: top + (y_high - y) / (y_high - y_low) * (height - top - bottom)
    parts = [f'<svg viewBox="0 0 {width} {height}" role="img" aria-label="{html.escape(y_label)}">']
    for y in _nice_ticks(y_low, y_high):
        parts.append(f'<line class="axis" x1="{left}" x2="{width - right}" y1="{sy(y):.1f}" y2="{sy(y):.1f}"/>'
                     f'<text x="{left - 6}" y="{sy(y) + 4:.1f}" text-anchor="end">{_tick_label(y)}</text>')
    for x in _nice_ticks(x_low, x_high, 8):
        parts.append(f'<text x="{sx(x):.1f}" y="{height - bottom + 16}" text-anchor="middle">{_tick_label(x)}</text>')
    for x, label in vlines:
        if x_low <= x <= x_high:
            parts.append(f'<line x1="{sx(x):.1f}" x2="{sx(x):.1f}" y1="{top}" y2="{height - bottom}" '
                         f'stroke="var(--muted)" stroke-dasharray="3 4"/>'
                         f'<text x="{sx(x) + 3:.1f}" y="{top + 10}">{html.escape(label)}</text>')
    for _, xs, ys, color in series:
        segments, current = [], []
        for x, y in zip(xs, ys):
            if y is None:
                if current:
                    segments.append(current)
                current = []
            else:
                current.append(f"{sx(x):.1f},{sy(y):.1f}")
        if current:
            segments.append(current)
        for segment in segments:
            parts.append(f'<polyline fill="none" stroke="{color}" stroke-width="1.6" points="{" ".join(segment)}"/>')
    parts.append(f'<text x="{(left + width - right) / 2}" y="{height - 4}" text-anchor="middle">{html.escape(x_label)}</text>'
                 f'<text x="12" y="{(top + height - bottom) / 2}" text-anchor="middle" '
                 f'transform="rotate(-90 12 {(top + height - bottom) / 2})">{html.escape(y_label)}</text>')
    if len(series) > 1:
        x = left + 8
        for name, _, _, color in series:
            parts.append(f'<rect x="{x}" y="{top + 2}" width="10" height="10" fill="{color}"/>'
                         f'<text x="{x + 14}" y="{top + 11}">{html.escape(name)}</text>')
            x += 18 + 7 * len(name)
    parts.append("</svg>")
    return "".join(parts)


def track_map(metrics, rows, size=640):
    """SVG top view (equal axes, ENU: x right, y up) of the track and the course."""
    scenario = metrics.get("scenario") or {}
    setpoints = metrics.get("setpoints") or []
    points = [(r["x_m"], r["y_m"]) for r in rows]
    for gate in scenario.get("gates", []):
        points += [tuple(gate["red"]), tuple(gate["green"])]
    for obstacle in scenario.get("obstacles", []):
        points.append(tuple(obstacle["position"]))
    for s in setpoints:
        points.append(tuple(s["target"]))
    if len(scenario.get("start", [])) >= 2:
        points.append(tuple(scenario["start"][:2]))
    if not points:
        return '<p class="note">No track recorded.</p>'
    x_low, x_high = min(p[0] for p in points) - 4, max(p[0] for p in points) + 4
    y_low, y_high = min(p[1] for p in points) - 4, max(p[1] for p in points) + 4
    span = max(x_high - x_low, y_high - y_low)
    cx, cy = (x_low + x_high) / 2, (y_low + y_high) / 2
    x_low, y_low = cx - span / 2, cy - span / 2
    margin = 36
    scale = (size - 2 * margin) / span
    sx = lambda x: margin + (x - x_low) * scale
    sy = lambda y: size - margin - (y - y_low) * scale
    parts = [f'<svg viewBox="0 0 {size} {size}" style="max-width:{size}px" role="img" aria-label="Top view">']
    for x in _nice_ticks(x_low, x_low + span, 8):
        parts.append(f'<line class="axis" x1="{sx(x):.1f}" x2="{sx(x):.1f}" y1="{margin}" y2="{size - margin}"/>'
                     f'<text x="{sx(x):.1f}" y="{size - margin + 16}" text-anchor="middle">{_tick_label(x)}</text>')
    for y in _nice_ticks(y_low, y_low + span, 8):
        parts.append(f'<line class="axis" x1="{margin}" x2="{size - margin}" y1="{sy(y):.1f}" y2="{sy(y):.1f}"/>'
                     f'<text x="{margin - 6}" y="{sy(y) + 4:.1f}" text-anchor="end">{_tick_label(y)}</text>')
    parts.append(f'<text x="{size / 2}" y="{size - 4}" text-anchor="middle">x east (m)</text>'
                 f'<text x="10" y="{size / 2}" text-anchor="middle" transform="rotate(-90 10 {size / 2})">y north (m)</text>')
    for gate in scenario.get("gates", []):
        parts.append(f'<line x1="{sx(gate["red"][0]):.1f}" y1="{sy(gate["red"][1]):.1f}" '
                     f'x2="{sx(gate["green"][0]):.1f}" y2="{sy(gate["green"][1]):.1f}" '
                     f'stroke="var(--muted)" stroke-dasharray="4 4"/>')
        for color in ("red", "green"):
            parts.append(f'<circle cx="{sx(gate[color][0]):.1f}" cy="{sy(gate[color][1]):.1f}" '
                         f'r="{max(3, gate["radius_m"] * scale):.1f}" fill="{GATE_COLORS[color]}"/>')
    for obstacle in scenario.get("obstacles", []):
        parts.append(f'<circle cx="{sx(obstacle["position"][0]):.1f}" cy="{sy(obstacle["position"][1]):.1f}" '
                     f'r="{max(3, obstacle["radius_m"] * scale):.1f}" fill="var(--muted)"/>')
    for s in setpoints:
        x, y = s["target"]
        color = OUTCOME_COLORS.get(s["outcome"], "#d97706")
        yaw = math.radians(s["commanded_heading_deg_enu"])
        length = max(2.0, s["tolerance_m"] * 1.5)
        parts.append(f'<circle cx="{sx(x):.1f}" cy="{sy(y):.1f}" r="{s["tolerance_m"] * scale:.1f}" '
                     f'fill="{color}" fill-opacity="0.18" stroke="{color}"/>')
        if s["heading_scored"]:
            parts.append(f'<line x1="{sx(x):.1f}" y1="{sy(y):.1f}" x2="{sx(x + length * math.cos(yaw)):.1f}" '
                         f'y2="{sy(y + length * math.sin(yaw)):.1f}" stroke="{color}" stroke-width="2"/>')
        parts.append(f'<text x="{sx(x) + 6:.1f}" y="{sy(y) - 6:.1f}" style="fill:{color}">'
                     f'{s["index"] + 1} {html.escape(s["name"])}</text>')
    if points and rows:
        track = " ".join(f"{sx(r['x_m']):.1f},{sy(r['y_m']):.1f}" for r in rows)
        parts.append(f'<polyline fill="none" stroke="var(--track)" stroke-width="1.6" points="{track}"/>')
        last = rows[-1]
        parts.append(f'<circle cx="{sx(last["x_m"]):.1f}" cy="{sy(last["y_m"]):.1f}" r="4" fill="var(--track)"/>')
    if len(scenario.get("start", [])) >= 2:
        x, y = scenario["start"][:2]
        parts.append(f'<rect x="{sx(x) - 5:.1f}" y="{sy(y) - 5:.1f}" width="10" height="10" fill="none" '
                     f'stroke="var(--track)" stroke-width="2"/><text x="{sx(x) + 8:.1f}" y="{sy(y) + 14:.1f}">start</text>')
    parts.append("</svg>")
    return "".join(parts)


def read_timeseries(path):
    """Rows of timeseries.csv as dicts of floats (None for empty cells)."""
    path = Path(path)
    if not path.is_file():
        return []
    rows = []
    with path.open(newline="") as stream:
        for raw in csv.DictReader(stream):
            rows.append({k: (float(v) if v not in ("", None) else None) for k, v in raw.items()})
    return rows


def render_report(metrics, rows, title=None):
    """Return the report page (HTML text) for a metrics dict and timeseries rows."""
    scenario = metrics.get("scenario") or {}
    status = metrics.get("status", "unknown")
    good = status in ("completed", "stopped")
    name = scenario.get("name", "run")
    title = title or f"Run report: {name}"
    setpoints = metrics.get("setpoints") or []
    out = [f'<!doctype html><html lang="en"><head><meta charset="utf-8">'
           f'<meta name="viewport" content="width=device-width, initial-scale=1">'
           f"<title>{html.escape(title)}</title><style>{STYLE}</style></head><body><main>",
           f"<h1>{html.escape(title)}</h1>",
           f'<p class="sub">{html.escape(metrics.get("course", scenario.get("kind", "gates")))} course · '
           f'{html.escape(str(metrics.get("mode", "race")))} · seed {html.escape(str(metrics.get("seed", "–")))} · '
           f'environment {html.escape(str(metrics.get("environment", "–")))} · state source '
           f'{html.escape(str(metrics.get("state_source", "–")))}</p>',
           f'<p><span class="badge" style="background:{"var(--ok)" if good else "var(--bad)"}">'
           f"{html.escape(status)}</span></p>"]
    kpis = [("Time", _fmt(metrics.get("time_s"), 1, " s")),
            ("Distance travelled", _fmt(metrics.get("distance_travelled_m"), 1, " m")),
            ("Min. clearance", _fmt(metrics.get("min_clearance_m"), 2, " m")),
            ("Collision", "unknown" if metrics.get("collision") is None else _fmt(metrics.get("collision")))]
    if metrics.get("course") == "setpoints" or setpoints:
        required = metrics.get("setpoints_required")
        kpis.insert(0, ("Targets reached", f'{metrics.get("setpoints_reached", 0)}'
                        + (f" of {required} required" if required is not None else
                           f' of {metrics.get("setpoints_issued", 0)}')))
    else:
        kpis.insert(0, ("Gates passed", f'{metrics.get("gates_passed", "–")} of {metrics.get("gates_total", "–")}'))
    out.append('<div class="kpis">' + "".join(
        f'<div class="kpi"><div class="label">{label}</div><div class="value">{value}</div></div>'
        for label, value in kpis) + "</div>")
    out.append("<h2>Top view</h2>" + track_map(metrics, rows))
    out.append('<p class="note">Track from simulation ground truth. Circles: target tolerance '
               "(green reached, red missed, blue replaced, amber active); lines: commanded heading.</p>")
    if setpoints:
        out.append("<h2>Targets</h2><div class=\"scroll\"><table><thead><tr>"
                   "<th>#</th><th>Name</th><th>Outcome</th><th>Issued (s)</th><th>Reach (s)</th>"
                   "<th>Settle (s)</th><th>Final error (m)</th><th>Heading error (°)</th>"
                   "<th>Hold RMS (m)</th><th>Hold RMS (°)</th><th>Overshoot (m)</th><th>Max cross-track (m)</th>"
                   "<th>Path efficiency</th><th>Thrust impulse (N·s)</th></tr></thead><tbody>")
        for s in setpoints:
            color = OUTCOME_COLORS.get(s["outcome"], "inherit")
            heading = _fmt(abs(s["final_heading_error_deg"]), 1) + ("" if s["heading_scored"] else " (free)")
            out.append(
                f"<tr><td>{s['index'] + 1}</td><td>{html.escape(s['name'])}</td>"
                f'<td style="color:{color};font-weight:600">{html.escape(s["outcome"])}'
                f'{"" if s["required"] else " (optional)"}</td>'
                f"<td>{_fmt(s['issued_at_s'], 1)}</td><td>{_fmt(s['time_to_reach_s'], 1)}</td>"
                f"<td>{_fmt(s['time_to_settle_s'], 1)}</td><td>{_fmt(s['final_distance_m'])}</td>"
                f"<td>{heading}</td><td>{_fmt(s['hold_rms_distance_m'])}</td>"
                f"<td>{_fmt(s['hold_rms_heading_error_deg'], 1)}</td><td>{_fmt(s['overshoot_m'])}</td>"
                f"<td>{_fmt(s['max_cross_track_m'])}</td><td>{_fmt(s['path_efficiency'])}</td>"
                f"<td>{_fmt(s['force_impulse_ns'], 0)}</td></tr>")
        out.append("</tbody></table></div>")
        out.append('<p class="note">Reach: first time within tolerance (position, and heading unless free). '
                   "Settle: start of the final hold. Hold RMS: error during that hold. Overshoot: largest distance "
                   "after first reaching the target. Path efficiency: straight line / travelled path. Thrust impulse: "
                   "integral of the summed commanded thrust magnitudes, not electrical energy.</p>")
    times = [r["t_s"] for r in rows]
    vlines = [(s["issued_at_s"], str(s["index"] + 1)) for s in setpoints]
    if rows and any(r.get("distance_m") is not None for r in rows):
        out.append("<h2>Error to the active target</h2>")
        out.append(line_chart([("distance", times, [r.get("distance_m") for r in rows], PALETTE[0])],
                              "time (s)", "distance (m)", vlines))
        out.append(line_chart([("heading error", times, [r.get("heading_error_deg") for r in rows], PALETTE[1])],
                              "time (s)", "heading error (°)", vlines))
    if rows:
        out.append("<h2>Motion</h2>")
        out.append(line_chart([("surge", times, [r.get("surge_mps") for r in rows], PALETTE[0]),
                               ("sway", times, [r.get("sway_mps") for r in rows], PALETTE[1])],
                              "time (s)", "speed (m/s)", vlines))
        forces = sorted((k for k in rows[0] if k.startswith("force_")), key=lambda k: int(k.split("_")[1]))
        if forces:
            out.append("<h2>Thrust passed by the guard</h2>")
            out.append(line_chart([(f"thruster {k.split('_')[1]}", times, [r.get(k) for r in rows],
                                    PALETTE[i % len(PALETTE)]) for i, k in enumerate(forces)],
                                  "time (s)", "force (N)", vlines))
    out.append("<h2>Provenance</h2><dl>")
    for key in ("label", "profile", "run_id", "git_commit", "runner_git_commit", "image_identity",
                "scenario_sha256", "manifest_sha256", "wall_time_s", "real_time_factor"):
        if key in metrics:
            out.append(f"<dt>{key}</dt><dd>{_fmt(metrics[key], 3) if isinstance(metrics[key], float) else html.escape(str(metrics[key]))}</dd>")
    out.append("</dl><p class=\"note\">Scores use simulation ground truth. Vessel models marked uncalibrated "
               "(e.g. munin_v0) are not evidence of the real boat.</p></main></body></html>")
    return "".join(out)


def write_report(metrics_path, timeseries_path, output_path):
    """Read the metrics and time series and write the report atomically."""
    metrics = json.loads(Path(metrics_path).read_text())
    text = render_report(metrics, read_timeseries(timeseries_path))
    output = Path(output_path)
    temporary = output.with_suffix(output.suffix + ".tmp")
    temporary.write_text(text)
    temporary.replace(output)
    return output
