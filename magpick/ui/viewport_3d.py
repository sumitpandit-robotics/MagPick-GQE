"""
viewport_3d.py

3D scene rendering for the MagPick-GQE Dashboard.
Plotly interactive mode + Open3D high-quality renders.
"""

import numpy as np
import plotly.graph_objects as go
from dash import dcc, html
import dash_bootstrap_components as dbc

from magpick.ui.utils import make_cylinder_mesh, make_arrow_trace, score_to_color, downsample_pcd


def make_3d_viewport():
    """Build the 3D viewport component."""
    return html.Div([
        dbc.Row([
            dbc.Col([
                dbc.ButtonGroup([
                    dbc.Button("Isometric", id="cam-iso", size="sm", color="secondary"),
                    dbc.Button("Top", id="cam-top", size="sm", color="secondary"),
                    dbc.Button("Front", id="cam-front", size="sm", color="secondary"),
                    dbc.Button("Side", id="cam-side", size="sm", color="secondary"),
                ], size="sm"),
            ], width="auto"),
            dbc.Col([
                html.Div(id="viewport-info", className="text-muted small align-self-center ms-2"),
            ]),
        ], className="mb-2"),
        dcc.Loading(
            dcc.Graph(
                id="scene-3d",
                style={"height": "50vh"},
                config={"displayModeBar": True, "scrollZoom": True},
            ),
            type="circle",
            id="viewport-loading",
        ),
    ])


def render_scene(point_cloud, billets, candidates, scores, selected_idx=None):
    """Render the 3D scene with point cloud, billets, and grasp candidates.

    Parameters
    ----------
    point_cloud : open3d.geometry.PointCloud or None
    billets : list of dict with 'position', 'radius', 'length'
    candidates : list of CandidatePose
    scores : list of float
    selected_idx : int or None
    """
    fig = go.Figure()

    # Point cloud
    if point_cloud is not None and not point_cloud.is_empty():
        pts = np.asarray(point_cloud.points)
        colors = None
        if point_cloud.has_colors():
            colors = np.asarray(point_cloud.colors)

        if colors is not None and len(colors) == len(pts):
            fig.add_trace(go.Scatter3d(
                x=pts[:, 0], y=pts[:, 1], z=pts[:, 2],
                mode="markers",
                marker=dict(size=1, color=colors, opacity=0.4),
                name="Scene",
                hoverinfo="skip",
            ))
        else:
            fig.add_trace(go.Scatter3d(
                x=pts[:, 0], y=pts[:, 1], z=pts[:, 2],
                mode="markers",
                marker=dict(size=1, color="lightgray", opacity=0.3),
                name="Scene",
                hoverinfo="skip",
            ))

    # Billets as cylinders
    for i, b in enumerate(billets):
        pos = b.get("position", [0, 0, 0])
        r = b.get("radius", 0.0325)
        h = b.get("length", 0.175)
        fig.add_trace(make_cylinder_mesh(pos, r, h, color="#8B4513", opacity=0.7))
        fig.add_trace(go.Scatter3d(
            x=[pos[0]], y=[pos[1]], z=[pos[2] + h / 2 + 0.02],
            mode="text",
            text=[f"Billet {b.get('id', i)}"],
            textfont=dict(size=10, color="#8B4513"),
            showlegend=False,
            hoverinfo="skip",
        ))

    # Grasp candidates as colored arrows
    for i, (cand, score) in enumerate(zip(candidates, scores)):
        color = score_to_color(score)
        width = 8 if i == selected_idx else 4
        opacity = 1.0 if i == selected_idx else 0.8

        # Position
        pos = np.asarray(cand.position)

        # Direction from orientation (z-axis = approach direction)
        direction = np.zeros(3)
        if cand.orientation is not None:
            from magpick.ui.utils import quat_to_direction
            direction = quat_to_direction(cand.orientation, "z")

        # Arrow trace
        fig.add_trace(make_arrow_trace(pos, direction, length=0.06, color=color, width=width,
                                       name=f"Candidate {i} (score={score:.2f})"))

        # Score label
        fig.add_trace(go.Scatter3d(
            x=[pos[0]], y=[pos[1]], z=[pos[2] + 0.08],
            mode="text",
            text=[f"#{i+1} {score:.2f}"],
            textfont=dict(size=9, color=color, family="monospace"),
            showlegend=False,
            hovertext=f"Candidate {i+1}<br>Score: {score:.3f}",
            hoverinfo="text",
        ))

    # Layout
    fig.update_layout(
        scene=dict(
            aspectmode="data",
            xaxis=dict(title="X (m)", showgrid=True, gridcolor="rgba(200,200,200,0.3)"),
            yaxis=dict(title="Y (m)", showgrid=True, gridcolor="rgba(200,200,200,0.3)"),
            zaxis=dict(title="Z (m)", showgrid=True, gridcolor="rgba(200,200,200,0.3)"),
            bgcolor="rgba(240,240,240,1)",
        ),
        margin=dict(l=0, r=0, t=0, b=0),
        showlegend=False,
        paper_bgcolor="white",
    )

    return fig


CAMERA_PRESETS = {
    "iso": dict(eye=dict(x=1.5, y=1.5, z=1.0), up=dict(x=0, y=0, z=1)),
    "top": dict(eye=dict(x=0, y=0, z=3.0), up=dict(x=0, y=1, z=0)),
    "front": dict(eye=dict(x=3.0, y=0, z=0), up=dict(x=0, y=0, z=1)),
    "side": dict(eye=dict(x=0, y=3.0, z=0), up=dict(x=0, y=0, z=1)),
}
