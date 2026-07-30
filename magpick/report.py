"""
report.py

Production-grade report generation for MagPick-GQE.

Generates an industrial engineering report with:
- Executive summary dashboard
- Per-candidate multi-view snapshots
- Radar charts for candidate comparison
- Force vector diagrams
- Progress bars for metrics
- Engineering recommendations
- Risk assessment

MRD 5.7 (reporting).
"""

import csv
import json
import os
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional

import numpy as np

from magpick.grasp_quality_engine import EvaluationReport
from magpick.models import CandidateResult
from magpick.utils.visualization import (
    render_force_vector_svg,
    render_progress_bar,
    compute_radar_data,
    generate_recommendations,
    HAS_OPEN3D,
    CAMERA_PRESETS,
)


def _convert(o):
    if isinstance(o, (np.bool_,)):
        return bool(o)
    if isinstance(o, (np.integer,)):
        return int(o)
    if isinstance(o, (np.floating,)):
        return float(o)
    if isinstance(o, np.ndarray):
        return o.tolist()
    if isinstance(o, dict):
        return {k: _convert(v) for k, v in o.items()}
    if isinstance(o, (list, tuple)):
        return [_convert(v) for v in o]
    return o


def _candidate_to_dict(result: CandidateResult) -> Dict:
    evaluations = []
    for ev in result.evaluator_results:
        evaluations.append({
            "name": ev.name,
            "passed": bool(ev.passed),
            "score": round(float(ev.score), 4),
            "weight": round(float(ev.weight), 4),
            "reason": ev.reason,
            "details": _convert(ev.details),
        })
    return {
        "rank": int(result.rank),
        "status": result.status,
        "final_score": round(float(result.final_score), 4),
        "evaluations": evaluations,
    }


# ==========================================================
# JSON Report
# ==========================================================

def generate_json_report(
    report: EvaluationReport,
    output_path: str = "output/evaluation_report.json",
) -> str:
    data = {
        "timestamp": datetime.now().isoformat(),
        "gripper": {
            "name": report.gripper.name,
            "max_force_N": float(report.gripper.max_force),
            "pad_width_m": float(report.gripper.pad_width),
            "pad_length_m": float(report.gripper.pad_length),
        },
        "billet": {
            "id": report.billet.id,
            "diameter_m": float(report.billet.radius * 2),
            "length_m": float(report.billet.length),
            "weight_kg": float(report.billet.weight),
            "material": report.billet.material,
            "surface": report.billet.surface,
        },
        "compatibility": {
            "compatible": bool(report.compatibility.compatible),
            "message": report.compatibility.message,
            "details": _convert(report.compatibility.details),
        },
        "summary": _convert(report.summary),
        "candidates": [_candidate_to_dict(r) for r in report.candidates],
    }

    Path(output_path).parent.mkdir(parents=True, exist_ok=True)
    with open(output_path, "w") as f:
        json.dump(data, f, indent=2, default=_convert)

    return output_path


# ==========================================================
# CSV Report
# ==========================================================

def generate_csv_report(
    report: EvaluationReport,
    output_path: str = "output/evaluation_report.csv",
) -> str:
    eval_names = []
    for r in report.candidates:
        for ev in r.evaluator_results:
            if ev.name not in eval_names:
                eval_names.append(ev.name)

    headers = ["rank", "status", "final_score"]
    for name in eval_names:
        headers.extend([f"{name}_score", f"{name}_passed"])

    rows = []
    for r in report.candidates:
        row = [r.rank, r.status, round(r.final_score, 4)]
        ev_map = {ev.name: ev for ev in r.evaluator_results}
        for name in eval_names:
            ev = ev_map.get(name)
            if ev:
                row.extend([round(ev.score, 4), ev.passed])
            else:
                row.extend(["", ""])
        rows.append(row)

    Path(output_path).parent.mkdir(parents=True, exist_ok=True)
    with open(output_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(headers)
        writer.writerows(rows)

    return output_path


# ==========================================================
# HTML Report — Production Grade
# ==========================================================

def generate_html_report(
    report: EvaluationReport,
    output_path: str = "output/evaluation_report.html",
    snapshot_dir: str = "output/snapshots",
    gripper_mesh_path: Optional[str] = None,
    scene=None,
    pole_layout=None,
) -> str:
    """Generate a production-grade HTML engineering report."""

    # --- Try to render multi-view snapshots ---
    snapshots = {}
    if gripper_mesh_path and HAS_OPEN3D:
        try:
            from magpick.utils.visualization import render_multi_view_snapshots
            for r in report.candidates:
                snaps = render_multi_view_snapshots(
                    candidate=r,
                    gripper_mesh_path=gripper_mesh_path,
                    scene=scene,
                    billet=report.billet,
                    pole_layout=pole_layout,
                )
                if snaps:
                    snapshots[r.rank] = snaps
        except Exception:
            pass  # Graceful fallback — snapshots are optional

    # --- Compute summary stats ---
    total = len(report.candidates)
    passed_count = sum(1 for c in report.candidates if c.final_score > 0)
    failed_count = total - passed_count
    pass_rate = (passed_count / total * 100) if total > 0 else 0
    best = report.best
    best_score = best.final_score if best else 0
    compat = report.compatibility

    # --- Collect evaluator names ---
    eval_names = []
    for r in report.candidates:
        for ev in r.evaluator_results:
            if ev.name not in eval_names:
                eval_names.append(ev.name)

    # --- Find recommended candidate (best passing) ---
    recommended = None
    for r in report.candidates:
        if r.final_score > 0:
            recommended = r
            break

    # --- Generate radar chart data ---
    radar_datasets = []
    colors = ["#3498db", "#e74c3c", "#2ecc71", "#f39c12", "#9b59b6", "#1abc9c"]
    for i, r in enumerate(report.candidates[:6]):
        rd = compute_radar_data(r.evaluator_results)
        radar_datasets.append({
            "label": f"Candidate #{r.rank} ({r.status})",
            "data": rd["scores"],
            "borderColor": colors[i % len(colors)],
            "backgroundColor": colors[i % len(colors)] + "20",
        })

    # --- Force diagrams for top candidates ---
    force_diagrams = {}
    for r in report.candidates[:5]:
        mag_ev = next((ev for ev in r.evaluator_results if ev.name == "Magnetic"), None)
        if mag_ev:
            d = mag_ev.details
            force_diagrams[r.rank] = render_force_vector_svg(
                holding_force=d.get("holding_force", 0),
                required_force=d.get("required_force", 0),
                safety_factor=d.get("safety_factor", 0),
            )

    # --- Progress bars for metrics ---
    def make_bars(ev):
        bars = []
        if ev.name == "Magnetic":
            sf = ev.details.get("safety_factor", 0)
            bars.append(render_progress_bar(sf, 4.0, "Safety Factor"))
        elif ev.name == "Contact Area":
            ratio = ev.details.get("coverage_ratio", 0)
            bars.append(render_progress_bar(ratio, 1.0, "Coverage"))
        elif ev.name == "Pole Coverage":
            ratio = ev.details.get("pole_coverage_ratio", 0)
            bars.append(render_progress_bar(ratio, 1.0, "Pole Coverage"))
        elif ev.name == "Collision":
            score = ev.details.get("clearance_score", 0)
            bars.append(render_progress_bar(score, 1.0, "Clearance"))
        elif ev.name == "Geometry":
            error = ev.details.get("normal_error_deg", 0)
            bars.append(render_progress_bar(max(0, 1 - error / 90), 1.0, "Normal Alignment"))
        elif ev.name == "Robot Dynamics":
            sf = ev.details.get("dynamic_safety_factor", 0)
            bars.append(render_progress_bar(min(sf, 5.0), 5.0, "Dynamic SF"))
        return "".join(bars)

    # --- Candidate rows ---
    candidate_sections = []
    for r in report.candidates:
        status_class = "pass" if r.final_score > 0 else "fail"
        status_icon = "&#10003;" if r.final_score > 0 else "&#10007;"

        # Evaluator table rows
        ev_rows = []
        ev_map = {ev.name: ev for ev in r.evaluator_results}
        for name in eval_names:
            ev = ev_map.get(name)
            if ev:
                cls = "pass" if ev.passed else "fail"
                icon = "&#10003;" if ev.passed else "&#10007;"
                bars = make_bars(ev)
                ev_rows.append(f"""
                <tr class="{cls}">
                    <td><span class="status-icon {cls}">{icon}</span> {ev.name}</td>
                    <td>{ev.score:.3f}</td>
                    <td>{bars}</td>
                    <td>{ev.reason}</td>
                </tr>""")

        # Recommendations
        recs = generate_recommendations(r.evaluator_results)
        rec_html = ""
        if recs:
            rec_items = []
            for rec in recs:
                risk_cls = rec["risk"].lower()
                rec_items.append(f"""
                <div class="rec-item">
                    <span class="rec-icon">{rec['icon']}</span>
                    <div class="rec-content">
                        <strong>{rec['evaluator']}</strong> \u2014 {rec['reason']}<br>
                        <em>Risk: <span class="risk-{risk_cls}">{rec['risk']}</span></em><br>
                        <em>Suggestion: {rec['suggestion']}</em>
                    </div>
                </div>""")
            rec_html = f"""
            <div class="recommendations">
                <h4>Engineering Recommendations</h4>
                {''.join(rec_items)}
            </div>"""

        # Force diagram
        force_svg = force_diagrams.get(r.rank, "")

        # Force diagram section
        force_section = '<div class="force-diagram"><h4>Force Analysis</h4>' + force_svg + '</div>' if force_svg else ''

        # Multi-view snapshots section
        snaps = snapshots.get(r.rank, {})
        if snaps:
            snap_items = []
            for view_name, b64 in snaps.items():
                label = CAMERA_PRESETS.get(view_name, {}).get("label", view_name)
                snap_items.append(
                    '<div class="snap-item">'
                    '<div class="snap-label">' + label + '</div>'
                    '<img src="data:image/png;base64,' + b64 + '" alt="' + view_name + '" />'
                    '</div>'
                )
            snapshots_html = (
                '<div class="snapshots-section">'
                '<h4>Multi-View Snapshots</h4>'
                '<div class="snap-grid">' + ''.join(snap_items) + '</div>'
                '</div>'
            )
        else:
            snapshots_html = ''

        # Radar chart for this candidate
        rd = compute_radar_data(r.evaluator_results)

        candidate_sections.append(f"""
        <div class="candidate-card {status_class}">
            <div class="candidate-header">
                <div class="candidate-title">
                    <h3>Candidate #{r.rank}</h3>
                    <span class="status-badge {status_class}">{r.status}</span>
                </div>
                <div class="candidate-score">
                    <div class="score-circle {status_class}">
                        <span class="score-value">{r.final_score:.2f}</span>
                        <span class="score-label">Score</span>
                    </div>
                </div>
            </div>

            <div class="candidate-body">
                <div class="evaluator-table">
                    <table>
                        <thead>
                            <tr>
                                <th>Evaluator</th>
                                <th>Score</th>
                                <th>Metric</th>
                                <th>Reason</th>
                            </tr>
                        </thead>
                        <tbody>
                            {''.join(ev_rows)}
                        </tbody>
                    </table>
                </div>

                <div class="candidate-details">
                    {force_section}

                    <div class="radar-container">
                        <canvas id="radar_{r.rank}" width="300" height="300"></canvas>
                    </div>
                </div>

                {rec_html}

                <!-- Multi-View Snapshots -->
                {snapshots_html}
            </div>
        </div>""")

    # --- Pre-compute chart scripts (avoid nested f-strings) ---
    radar_scripts_parts = []
    for r in report.candidates[:6]:
        labels = json.dumps([ev.name for ev in r.evaluator_results])
        data = json.dumps([round(float(ev.score), 3) for ev in r.evaluator_results])
        color = colors[r.rank % len(colors)]
        radar_scripts_parts.append(
            "new Chart(document.getElementById('radar_" + str(r.rank) + "'), {\n"
            "    type: 'radar',\n"
            "    data: {\n"
            "        labels: " + labels + ",\n"
            "        datasets: [{\n"
            "            label: 'Score',\n"
            "            data: " + data + ",\n"
            "            borderColor: '" + color + "',\n"
            "            backgroundColor: '" + color + "20',\n"
            "            pointRadius: 3,\n"
            "        }]\n"
            "    },\n"
            "    options: {\n"
            "        responsive: false,\n"
            "        scales: { r: { min: 0, max: 1, ticks: { stepSize: 0.2 } } },\n"
            "        plugins: { legend: { display: false } },\n"
            "    }\n"
            "});"
        )
    radar_scripts = "\n\n".join(radar_scripts_parts)

    comp_labels = json.dumps(["#" + str(r.rank) for r in report.candidates])
    comp_data = json.dumps([round(float(r.final_score), 3) for r in report.candidates])
    comp_colors = json.dumps(["#27ae60" if r.final_score > 0 else "#e74c3c" for r in report.candidates])
    comparison_script = (
        "new Chart(document.getElementById('comparisonChart'), {\n"
        "    type: 'bar',\n"
        "    data: {\n"
        "        labels: " + comp_labels + ",\n"
        "        datasets: [{\n"
        "            label: 'Final Score',\n"
        "            data: " + comp_data + ",\n"
        "            backgroundColor: " + comp_colors + ",\n"
        "            borderRadius: 4,\n"
        "        }]\n"
        "    },\n"
        "    options: {\n"
        "        responsive: true,\n"
        "        scales: { y: { min: 0, max: 1 } },\n"
        "        plugins: {\n"
        "            legend: { display: false },\n"
        "            title: { display: true, text: 'Candidate Scores (sorted by rank)', font: { size: 14 } }\n"
        "        }\n"
        "    }\n"
        "});"
    )

    # --- Build HTML ---
    html = f"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>MagPick-GQE \u2014 Industrial Grasp Evaluation Report</title>
<script src="https://cdn.jsdelivr.net/npm/chart.js@4.4.0/dist/chart.umd.min.js"></script>
<style>
    :root {{
        --pass: #27ae60;
        --fail: #e74c3c;
        --warn: #f39c12;
        --bg: #f0f2f5;
        --card: #ffffff;
        --border: #e0e0e0;
        --text: #2c3e50;
        --text-light: #7f8c8d;
        --accent: #3498db;
    }}
    * {{ margin: 0; padding: 0; box-sizing: border-box; }}
    body {{
        font-family: 'Segoe UI', -apple-system, BlinkMacSystemFont, sans-serif;
        background: var(--bg);
        color: var(--text);
        line-height: 1.6;
    }}

    /* Executive Summary Header */
    .exec-header {{
        background: linear-gradient(135deg, #1a1a2e 0%, #16213e 50%, #0f3460 100%);
        color: white;
        padding: 40px;
        text-align: center;
    }}
    .exec-header h1 {{
        font-size: 28px;
        font-weight: 300;
        letter-spacing: 2px;
        margin-bottom: 5px;
    }}
    .exec-header .subtitle {{
        font-size: 14px;
        color: #95a5a6;
        text-transform: uppercase;
        letter-spacing: 3px;
    }}

    /* Dashboard Cards */
    .dashboard {{
        display: grid;
        grid-template-columns: repeat(auto-fit, minmax(200px, 1fr));
        gap: 16px;
        padding: 24px 40px;
        max-width: 1200px;
        margin: 0 auto;
    }}
    .dash-card {{
        background: var(--card);
        border-radius: 12px;
        padding: 20px;
        box-shadow: 0 2px 8px rgba(0,0,0,0.06);
        text-align: center;
        border-top: 3px solid var(--accent);
    }}
    .dash-card.pass {{ border-top-color: var(--pass); }}
    .dash-card.fail {{ border-top-color: var(--fail); }}
    .dash-card.warn {{ border-top-color: var(--warn); }}
    .dash-card .card-value {{
        font-size: 32px;
        font-weight: 700;
        margin: 8px 0 4px;
    }}
    .dash-card .card-label {{
        font-size: 12px;
        color: var(--text-light);
        text-transform: uppercase;
        letter-spacing: 1px;
    }}

    /* Project Info */
    .project-info {{
        max-width: 1200px;
        margin: 0 auto;
        padding: 0 40px 24px;
    }}
    .info-grid {{
        display: grid;
        grid-template-columns: 1fr 1fr;
        gap: 16px;
        background: var(--card);
        border-radius: 12px;
        padding: 24px;
        box-shadow: 0 2px 8px rgba(0,0,0,0.06);
    }}
    .info-item {{
        display: flex;
        justify-content: space-between;
        padding: 8px 0;
        border-bottom: 1px solid var(--border);
    }}
    .info-item:last-child {{ border-bottom: none; }}
    .info-label {{ color: var(--text-light); font-size: 13px; }}
    .info-value {{ font-weight: 600; font-size: 13px; }}

    /* Section */
    .section {{
        max-width: 1200px;
        margin: 0 auto;
        padding: 0 40px 24px;
    }}
    .section h2 {{
        font-size: 20px;
        font-weight: 600;
        margin-bottom: 16px;
        padding-bottom: 8px;
        border-bottom: 2px solid var(--accent);
    }}

    /* Candidate Cards */
    .candidate-card {{
        background: var(--card);
        border-radius: 12px;
        margin-bottom: 20px;
        box-shadow: 0 2px 8px rgba(0,0,0,0.06);
        overflow: hidden;
    }}
    .candidate-header {{
        display: flex;
        justify-content: space-between;
        align-items: center;
        padding: 16px 24px;
        border-bottom: 1px solid var(--border);
    }}
    .candidate-card.pass .candidate-header {{
        background: linear-gradient(90deg, #d4edda 0%, transparent 100%);
    }}
    .candidate-card.fail .candidate-header {{
        background: linear-gradient(90deg, #f8d7da 0%, transparent 100%);
    }}
    .candidate-title {{
        display: flex;
        align-items: center;
        gap: 12px;
    }}
    .candidate-title h3 {{ font-size: 18px; }}
    .status-badge {{
        padding: 4px 12px;
        border-radius: 20px;
        font-size: 12px;
        font-weight: 700;
        text-transform: uppercase;
    }}
    .status-badge.pass {{ background: var(--pass); color: white; }}
    .status-badge.fail {{ background: var(--fail); color: white; }}

    .score-circle {{
        width: 64px;
        height: 64px;
        border-radius: 50%;
        display: flex;
        flex-direction: column;
        align-items: center;
        justify-content: center;
        border: 3px solid;
    }}
    .score-circle.pass {{ border-color: var(--pass); }}
    .score-circle.fail {{ border-color: var(--fail); }}
    .score-value {{ font-size: 20px; font-weight: 700; line-height: 1; }}
    .score-label {{ font-size: 9px; color: var(--text-light); text-transform: uppercase; }}

    .candidate-body {{ padding: 20px 24px; }}

    .evaluator-table table {{
        width: 100%;
        border-collapse: collapse;
        margin-bottom: 16px;
    }}
    .evaluator-table th, .evaluator-table td {{
        padding: 10px 12px;
        text-align: left;
        border-bottom: 1px solid var(--border);
        font-size: 13px;
    }}
    .evaluator-table th {{
        background: #f8f9fa;
        font-weight: 600;
        text-transform: uppercase;
        font-size: 11px;
        letter-spacing: 0.5px;
        color: var(--text-light);
    }}
    .evaluator-table tr.pass td {{ border-left: 3px solid var(--pass); }}
    .evaluator-table tr.fail td {{ border-left: 3px solid var(--fail); }}

    .status-icon {{ font-weight: bold; margin-right: 4px; }}
    .status-icon.pass {{ color: var(--pass); }}
    .status-icon.fail {{ color: var(--fail); }}

    .candidate-details {{
        display: grid;
        grid-template-columns: 1fr 1fr;
        gap: 16px;
        margin-top: 16px;
    }}
    .force-diagram, .radar-container {{
        background: #f8f9fa;
        border-radius: 8px;
        padding: 16px;
    }}
    .radar-container canvas {{ max-width: 300px; margin: 0 auto; display: block; }}

    .recommendations {{
        margin-top: 16px;
        background: #fff8e1;
        border-radius: 8px;
        padding: 16px;
        border-left: 4px solid var(--warn);
    }}
    .recommendations h4 {{ margin-bottom: 8px; font-size: 14px; }}
    .rec-item {{
        display: flex;
        gap: 8px;
        margin-bottom: 8px;
        font-size: 13px;
        line-height: 1.5;
    }}
    .rec-icon {{ font-size: 16px; flex-shrink: 0; }}
    .risk-high {{ color: var(--fail); font-weight: 700; }}
    .risk-medium {{ color: var(--warn); font-weight: 700; }}
    .risk-low {{ color: var(--pass); font-weight: 700; }}

    /* Multi-View Snapshots */
    .snapshots-section {{
        margin-top: 16px;
        background: #f8f9fa;
        border-radius: 8px;
        padding: 16px;
    }}
    .snapshots-section h4 {{ margin-bottom: 12px; font-size: 14px; }}
    .snap-grid {{
        display: grid;
        grid-template-columns: repeat(auto-fit, minmax(200px, 1fr));
        gap: 12px;
    }}
    .snap-item {{
        text-align: center;
    }}
    .snap-item img {{
        width: 100%;
        border-radius: 6px;
        border: 1px solid var(--border);
    }}
    .snap-label {{
        font-size: 11px;
        color: var(--text-light);
        text-transform: uppercase;
        letter-spacing: 0.5px;
        margin-bottom: 4px;
    }}

    /* Compatibility Banner */
    .compat-banner {{
        max-width: 1200px;
        margin: 0 auto 24px;
        padding: 0 40px;
    }}
    .compat-inner {{
        border-radius: 12px;
        padding: 20px 24px;
        display: flex;
        align-items: center;
        gap: 16px;
    }}
    .compat-inner.pass {{
        background: #d4edda;
        border: 1px solid #c3e6cb;
    }}
    .compat-inner.fail {{
        background: #f8d7da;
        border: 1px solid #f5c6cb;
    }}
    .compat-icon {{ font-size: 32px; }}
    .compat-text h3 {{ font-size: 16px; margin-bottom: 4px; }}
    .compat-text p {{ font-size: 13px; color: #555; }}

    /* Footer */
    .footer {{
        text-align: center;
        padding: 24px;
        color: var(--text-light);
        font-size: 12px;
    }}

    @media (max-width: 768px) {{
        .dashboard {{ grid-template-columns: repeat(2, 1fr); }}
        .info-grid {{ grid-template-columns: 1fr; }}
        .candidate-details {{ grid-template-columns: 1fr; }}
    }}
</style>
</head>
<body>

<!-- Executive Summary Header -->
<div class="exec-header">
    <h1>MAGPICK-GQE</h1>
    <div class="subtitle">Industrial Grasp Evaluation Report</div>
</div>

<!-- Dashboard Cards -->
<div class="dashboard">
    <div class="dash-card pass">
        <div class="card-label">Best Score</div>
        <div class="card-value">{best_score:.2f}</div>
    </div>
    <div class="dash-card {'pass' if compat.compatible else 'fail'}">
        <div class="card-label">Compatibility</div>
        <div class="card-value">{'PASS' if compat.compatible else 'FAIL'}</div>
    </div>
    <div class="dash-card">
        <div class="card-label">Candidates</div>
        <div class="card-value">{total}</div>
    </div>
    <div class="dash-card pass">
        <div class="card-label">Accepted</div>
        <div class="card-value">{passed_count}</div>
    </div>
    <div class="dash-card fail">
        <div class="card-label">Rejected</div>
        <div class="card-value">{failed_count}</div>
    </div>
    <div class="dash-card">
        <div class="card-label">Pass Rate</div>
        <div class="card-value">{pass_rate:.0f}%</div>
    </div>
</div>

<!-- Project Info -->
<div class="project-info">
    <div class="info-grid">
        <div>
            <div class="info-item">
                <span class="info-label">Gripper</span>
                <span class="info-value">{report.gripper.name}</span>
            </div>
            <div class="info-item">
                <span class="info-label">Rated Force</span>
                <span class="info-value">{report.gripper.max_force:.0f} N</span>
            </div>
            <div class="info-item">
                <span class="info-label">Pad Size</span>
                <span class="info-value">{report.gripper.pad_width*1000:.0f} \u00d7 {report.gripper.pad_length*1000:.0f} mm</span>
            </div>
        </div>
        <div>
            <div class="info-item">
                <span class="info-label">Billet</span>
                <span class="info-value">\u00d8{report.billet.radius*2*1000:.0f} \u00d7 {report.billet.length*1000:.0f} mm</span>
            </div>
            <div class="info-item">
                <span class="info-label">Material</span>
                <span class="info-value">{report.billet.material}</span>
            </div>
            <div class="info-item">
                <span class="info-label">Weight</span>
                <span class="info-value">{report.billet.weight:.2f} kg</span>
            </div>
        </div>
    </div>
</div>

<!-- Compatibility Banner -->
<div class="compat-banner">
    <div class="compat-inner {'pass' if compat.compatible else 'fail'}">
        <div class="compat-icon">{'&#9989;' if compat.compatible else '&#10060;'}</div>
        <div class="compat-text">
            <h3>{'Compatibility: PASS' if compat.compatible else 'Compatibility: FAIL'}</h3>
            <p>{compat.message}</p>
        </div>
    </div>
</div>

<!-- Recommended Candidate -->
{'<div class="section"><h2>Recommended Candidate</h2>' + candidate_sections[recommended.rank - 1] + '</div>' if recommended else ''}

<!-- Candidate Comparison Chart -->
<div class="section">
    <h2>Candidate Comparison</h2>
    <div style="background: var(--card); border-radius: 12px; padding: 24px; box-shadow: 0 2px 8px rgba(0,0,0,0.06);">
        <canvas id="comparisonChart" height="300"></canvas>
    </div>
</div>

<!-- All Candidates -->
<div class="section">
    <h2>All Candidates</h2>
    {''.join(candidate_sections)}
</div>

<!-- Footer -->
<div class="footer">
    Generated by MagPick-GQE v1.0 &mdash; {datetime.now().strftime("%Y-%m-%d %H:%M:%S")}
</div>

<script>
// Radar charts for each candidate
{radar_scripts}

// Comparison bar chart
{comparison_script}
</script>
</body>
</html>"""

    Path(output_path).parent.mkdir(parents=True, exist_ok=True)
    with open(output_path, "w") as f:
        f.write(html)

    return output_path


# ==========================================================
# Generate All Reports
# ==========================================================

def generate_report(
    evaluation_report: EvaluationReport,
    output_dir: str = "output",
    formats: Optional[List[str]] = None,
    snapshot_dir: Optional[str] = None,
    gripper_mesh_path: Optional[str] = None,
    scene=None,
    pole_layout=None,
) -> Dict[str, str]:
    if formats is None:
        formats = ["html", "json", "csv"]

    if snapshot_dir is None:
        snapshot_dir = f"{output_dir}/snapshots"

    paths = {}

    if "json" in formats:
        paths["json"] = generate_json_report(
            evaluation_report,
            output_path=f"{output_dir}/evaluation_report.json",
        )

    if "csv" in formats:
        paths["csv"] = generate_csv_report(
            evaluation_report,
            output_path=f"{output_dir}/evaluation_report.csv",
        )

    if "html" in formats:
        paths["html"] = generate_html_report(
            evaluation_report,
            output_path=f"{output_dir}/evaluation_report.html",
            snapshot_dir=snapshot_dir,
            gripper_mesh_path=gripper_mesh_path,
            scene=scene,
            pole_layout=pole_layout,
        )

    return paths
