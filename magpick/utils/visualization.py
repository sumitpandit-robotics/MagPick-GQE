"""
visualization.py

Open3D-based visualisation of grasp candidates.

Generates multi-view snapshots (top, side, front, isometric) for
each candidate, plus force-vector SVG diagrams and contact-coverage
progress bars.  All outputs are base64-encoded for embedding in HTML
reports — no external file dependencies.

MRD 5.8 (visualisation).
"""

import base64
import io
import math
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np

try:
    import open3d as o3d
    import open3d.visualization.rendering as rendering
    HAS_OPEN3D = True
except ImportError:
    HAS_OPEN3D = False


# ==========================================================
# Camera presets for multi-view rendering
# ==========================================================

CAMERA_PRESETS = {
    "isometric": {
        "eye": [0.4, 0.3, 0.4],
        "center_offset": [0.0, 0.0, 0.0],
        "up": [0.0, 0.0, 1.0],
        "label": "Isometric View",
    },
    "top": {
        "eye": [0.0, 0.0, 0.6],
        "center_offset": [0.0, 0.0, 0.0],
        "up": [0.0, 1.0, 0.0],
        "label": "Top View",
    },
    "front": {
        "eye": [0.6, 0.0, 0.0],
        "center_offset": [0.0, 0.0, 0.0],
        "up": [0.0, 0.0, 1.0],
        "label": "Front View",
    },
    "side": {
        "eye": [0.0, 0.6, 0.0],
        "center_offset": [0.0, 0.0, 0.0],
        "up": [0.0, 0.0, 1.0],
        "label": "Side View",
    },
}


def _ensure_mesh(mesh_path: str):
    """Load a mesh, returning None if unavailable."""
    if not HAS_OPEN3D:
        return None
    path = Path(mesh_path)
    if not path.exists():
        return None
    mesh = o3d.io.read_triangle_mesh(str(path))
    if mesh.is_empty():
        return None
    mesh.compute_vertex_normals()
    return mesh


def _make_cylinder(radius: float, length: float, color=None):
    """Create a cylinder mesh for billet approximation."""
    if not HAS_OPEN3D:
        return None
    mesh = o3d.geometry.TriangleMesh.create_cylinder(radius=radius, height=length)
    if color is not None:
        mesh.paint_uniform_color(color)
    mesh.compute_vertex_normals()
    return mesh


def _make_sphere(radius: float, color=None):
    """Create a sphere mesh for pole visualization."""
    if not HAS_OPEN3D:
        return None
    mesh = o3d.geometry.TriangleMesh.create_sphere(radius=radius)
    if color is not None:
        mesh.paint_uniform_color(color)
    mesh.compute_vertex_normals()
    return mesh


def _make_coordinate_frame(size=0.03):
    """Create a small coordinate frame."""
    if not HAS_OPEN3D:
        return None
    return o3d.geometry.TriangleMesh.create_coordinate_frame(size=size)


def render_multi_view_snapshots(
    candidate,
    gripper_mesh_path: str,
    scene=None,
    billet=None,
    pole_layout=None,
    width: int = 640,
    height: int = 480,
    focal_length: float = 400.0,
) -> Dict[str, str]:
    """Render multi-view PNG snapshots and return as base64 strings.

    Parameters
    ----------
    candidate : CandidateResult or CandidatePose
        The candidate to render.
    gripper_mesh_path : str
        Path to the gripper CAD mesh.
    scene : Scene, optional
        The point cloud scene.
    billet : Billet, optional
        Billet geometry.
    pole_layout : PoleLayout, optional
        Pole positions for visualization.
    width, height : int
        Render resolution.

    Returns
    -------
    dict
        {view_name: base64_encoded_png}
    """
    if not HAS_OPEN3D:
        return {}

    # Extract pose
    if hasattr(candidate, "candidate"):
        pose = candidate.candidate
    else:
        pose = candidate

    T = np.eye(4)
    from scipy.spatial.transform import Rotation
    R = Rotation.from_quat(pose.orientation).as_matrix()
    T[:3, :3] = R
    T[:3, 3] = pose.position

    results = {}

    for view_name, cam in CAMERA_PRESETS.items():
        img_b64 = _render_single_view(
            pose=pose,
            T=T,
            gripper_mesh_path=gripper_mesh_path,
            scene=scene,
            billet=billet,
            pole_layout=pole_layout,
            camera=cam,
            width=width,
            height=height,
            focal_length=focal_length,
        )
        if img_b64:
            results[view_name] = img_b64

    return results


def _render_single_view(
    pose, T, gripper_mesh_path, scene, billet, pole_layout,
    camera, width, height, focal_length,
) -> Optional[str]:
    """Render a single view and return base64-encoded PNG."""
    if not HAS_OPEN3D:
        return None

    try:
        renderer = rendering.OffscreenRenderer(width, height)
    except Exception:
        return None

    # Background
    renderer.scene.set_background([0.12, 0.12, 0.15, 1.0])

    # Lighting
    renderer.scene.scene.set_sun_light(
        direction=[0.5, 0.3, -1.0],
        intensity=75000,
        color=[1.0, 1.0, 1.0],
    )
    renderer.scene.scene.enable_sun_light(True)

    # Gripper mesh
    gripper_mesh = _ensure_mesh(gripper_mesh_path)
    if gripper_mesh is not None:
        gripper_tf = gripper_mesh.transform(T)
        mat = rendering.MaterialRecord()
        mat.shader = "defaultLit"
        mat.base_color = [0.2, 0.5, 0.85, 1.0]
        renderer.scene.add_geometry("gripper", gripper_tf, mat)

    # Point cloud
    if scene is not None and hasattr(scene, "point_cloud"):
        pcd = scene.point_cloud
        if not pcd.is_empty():
            mat_pcd = rendering.MaterialRecord()
            mat_pcd.shader = "defaultLit"
            mat_pcd.base_color = [0.65, 0.65, 0.65, 1.0]
            renderer.scene.add_geometry("scene", pcd, mat_pcd)

    # Billet cylinder
    if billet is not None:
        cyl = _make_cylinder(billet.radius, billet.length, color=[0.7, 0.45, 0.2])
        if cyl is not None:
            mat_cyl = rendering.MaterialRecord()
            mat_cyl.shader = "defaultLit"
            renderer.scene.add_geometry("billet", cyl, mat_cyl)

    # Pole locations (small red spheres)
    if pole_layout is not None:
        for px, py in pole_layout.pole_positions_m:
            sphere = _make_sphere(0.005, color=[1.0, 0.2, 0.2])
            if sphere is not None:
                T_pole = np.eye(4)
                T_pole[:3, 3] = [px, py, 0.0]
                T_pole = T @ T_pole
                sphere_tf = sphere.transform(T_pole)
                mat_pole = rendering.MaterialRecord()
                mat_pole.shader = "defaultLit"
                renderer.scene.add_geometry(f"pole_{px}_{py}", sphere_tf, mat_pole)

    # Coordinate frame at TCP
    coord = _make_coordinate_frame(size=0.025)
    if coord is not None:
        coord_tf = coord.transform(T)
        mat_coord = rendering.MaterialRecord()
        mat_coord.shader = "defaultLit"
        renderer.scene.add_geometry("tcp", coord_tf, mat_coord)

    # Camera
    center = (pose.position + np.array(camera["center_offset"])).tolist()
    eye = [pose.position[i] + camera["eye"][i] for i in range(3)]
    up = camera["up"]
    renderer.setup_camera(focal_length, center, eye, up)

    # Render
    img = renderer.render_to_image()
    try:
        renderer.release()
    except AttributeError:
        pass  # Open3D 0.19+ doesn't have release()

    # Encode to base64
    import tempfile, os
    with tempfile.NamedTemporaryFile(suffix=".png", delete=False) as f:
        tmp_path = f.name
    o3d.io.write_image(tmp_path, img)
    with open(tmp_path, "rb") as f:
        b64 = base64.b64encode(f.read()).decode("utf-8")
    os.unlink(tmp_path)

    return b64


# ==========================================================
# SVG Force Vector Diagram
# ==========================================================

def render_force_vector_svg(
    holding_force: float,
    required_force: float,
    safety_factor: float,
    width: int = 400,
    height: int = 120,
) -> str:
    """Generate an SVG force-vector comparison diagram.

    Shows holding force vs required force as proportional arrows,
    with color coding (green = safe, yellow = marginal, red = fail).

    Returns
    -------
    str
        SVG markup as a string.
    """
    max_force = max(holding_force, required_force, 1.0)
    bar_max = width - 120  # leave room for labels

    holding_len = int((holding_force / max_force) * bar_max)
    required_len = int((required_force / max_force) * bar_max)

    # Color based on safety factor
    if safety_factor >= 3.0:
        color = "#27ae60"  # green
    elif safety_factor >= 2.0:
        color = "#2ecc71"  # light green
    elif safety_factor >= 1.5:
        color = "#f39c12"  # yellow
    elif safety_factor >= 1.0:
        color = "#e67e22"  # orange
    else:
        color = "#e74c3c"  # red

    svg = f"""<svg width="{width}" height="{height}" xmlns="http://www.w3.org/2000/svg">
  <style>
    .label {{ font: 11px 'Segoe UI', sans-serif; fill: #333; }}
    .value {{ font: bold 12px 'Segoe UI', sans-serif; fill: #333; }}
    .arrow {{ stroke-width: 0; }}
  </style>

  <!-- Holding Force -->
  <text x="5" y="35" class="label">Holding</text>
  <rect x="70" y="20" width="{holding_len}" height="20" rx="4" fill="{color}" opacity="0.85"/>
  <polygon points="{70 + holding_len},15 {70 + holding_len + 10},30 {70 + holding_len},45" fill="{color}"/>
  <text x="{70 + holding_len + 15}" y="35" class="value">{holding_force:.0f}N</text>

  <!-- Required Force -->
  <text x="5" y="80" class="label">Required</text>
  <rect x="70" y="65" width="{required_len}" height="20" rx="4" fill="#7f8c8d" opacity="0.7"/>
  <polygon points="{70 + required_len},60 {70 + required_len + 10},75 {70 + required_len},90" fill="#7f8c8d"/>
  <text x="{70 + required_len + 15}" y="80" class="value">{required_force:.0f}N</text>

  <!-- Safety Factor -->
  <text x="5" y="115" class="label">Safety Factor</text>
  <text x="100" y="115" class="value" fill="{color}">{safety_factor:.2f}x</text>
</svg>"""

    return svg


# ==========================================================
# Progress Bar SVG
# ==========================================================

def render_progress_bar(
    value: float,
    max_value: float = 1.0,
    label: str = "",
    width: int = 200,
    height: int = 24,
    green_threshold: float = 0.7,
    yellow_threshold: float = 0.5,
) -> str:
    """Generate an SVG progress bar with color coding.

    Returns
    -------
    str
        SVG markup.
    """
    ratio = min(value / max_value, 1.0) if max_value > 0 else 0.0

    if ratio >= green_threshold:
        color = "#27ae60"
    elif ratio >= yellow_threshold:
        color = "#f39c12"
    else:
        color = "#e74c3c"

    bar_width = int(ratio * (width - 80))

    return f"""<svg width="{width}" height="{height}" xmlns="http://www.w3.org/2000/svg">
  <text x="0" y="16" font="11px 'Segoe UI', sans-serif" fill="#555">{label}</text>
  <rect x="70" y="4" width="{width - 80}" height="14" rx="3" fill="#ecf0f1"/>
  <rect x="70" y="4" width="{bar_width}" height="14" rx="3" fill="{color}"/>
  <text x="{width - 8}" y="16" font="bold 11px 'Segoe UI', sans-serif" fill="#333" text-anchor="end">{ratio*100:.0f}%</text>
</svg>"""


# ==========================================================
# Radar Chart Data (for Chart.js)
# ==========================================================

def compute_radar_data(evaluator_results: list) -> Dict:
    """Convert evaluator results to radar chart data for Chart.js.

    Returns
    -------
    dict
        {labels: [...], scores: [...], weights: [...]}
    """
    labels = []
    scores = []
    weights = []
    for ev in evaluator_results:
        labels.append(ev.name)
        scores.append(round(float(ev.score), 3))
        weights.append(round(float(ev.weight), 3))

    return {
        "labels": labels,
        "scores": scores,
        "weights": weights,
    }


# ==========================================================
# Recommendation Engine
# ==========================================================

def generate_recommendations(evaluator_results: list) -> List[Dict]:
    """Generate actionable recommendations based on evaluation results.

    Returns a list of recommendation dicts, each with:
    - evaluator: which evaluator failed
    - reason: human-readable explanation
    - risk: "High" / "Medium" / "Low"
    - suggestion: what to fix
    - icon: emoji-style indicator
    """
    recs = []

    for ev in evaluator_results:
        if ev.passed:
            continue

        name = ev.name
        details = ev.details

        if name == "Geometry":
            error = details.get("normal_error_deg", 0)
            recs.append({
                "evaluator": name,
                "reason": f"Approach angle {error:.1f}\u00b0 exceeds {15.0}\u00b0 limit",
                "risk": "High" if error > 30 else "Medium",
                "suggestion": "Rotate grasp pose to align approach vector with surface normal",
                "icon": "\u26a0\ufe0f",
            })

        elif name == "Magnetic":
            sf = details.get("safety_factor", 0)
            mat = details.get("material_recognized", True)
            if not mat:
                recs.append({
                    "evaluator": name,
                    "reason": f"Material '{details.get('material', '?')}' is non-ferromagnetic or unrecognized",
                    "risk": "High",
                    "suggestion": "Verify billet material or select a different gripper type",
                    "icon": "\u274c",
                })
            else:
                recs.append({
                    "evaluator": name,
                    "reason": f"Safety factor {sf:.2f} below minimum 2.0",
                    "risk": "High" if sf < 1.0 else "Medium",
                    "suggestion": "Reduce billet weight, increase grip area, or use higher-force gripper",
                    "icon": "\u26a0\ufe0f",
                })

        elif name == "Contact Area":
            ratio = details.get("coverage_ratio", 0)
            recs.append({
                "evaluator": name,
                "reason": f"Contact coverage only {ratio*100:.0f}% (minimum 85%)",
                "risk": "Medium" if ratio > 0.5 else "High",
                "suggestion": "Reposition grasp to centre on billet, or use larger pad gripper",
                "icon": "\u26a0\ufe0f",
            })

        elif name == "Pole Coverage":
            ratio = details.get("pole_coverage_ratio", 0)
            poles = details.get("poles_in_contact", 0)
            total = details.get("num_poles", 0)
            recs.append({
                "evaluator": name,
                "reason": f"Pole coverage {ratio*100:.0f}% ({poles}/{total} poles, minimum 60%)",
                "risk": "Medium" if ratio > 0.3 else "High",
                "suggestion": "Rotate grasp by ~18\u00b0 to centre poles on billet contact zone",
                "icon": "\u26a0\ufe0f",
            })

        elif name == "Collision":
            score = details.get("clearance_score", 0)
            recs.append({
                "evaluator": name,
                "reason": f"Clearance score {score:.2f} below minimum 0.50 — nearby obstacles detected",
                "risk": "High",
                "suggestion": "Choose an alternative grasp pose with more clearance",
                "icon": "\u274c",
            })

        elif name == "Robot Dynamics":
            margin = details.get("payload_margin", 0)
            recs.append({
                "evaluator": name,
                "reason": f"Payload margin {margin:.2f}x below minimum 1.2x",
                "risk": "Medium",
                "suggestion": "Verify robot payload capacity for this billet mass",
                "icon": "\u26a0\ufe0f",
            })

    return recs
