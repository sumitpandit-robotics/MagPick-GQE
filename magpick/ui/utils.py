"""
utils.py

Utility functions for the MagPick-GQE Dashboard.
Cylinder mesh generation, color scales, point cloud downsampling.
"""

import numpy as np
import plotly.graph_objects as go
import plotly.colors as pc


# ==========================================================
# Score → Color
# ==========================================================

def score_to_color(score):
    """Map a [0,1] score to a hex color (red → yellow → green)."""
    if score > 0.7:
        return "#27ae60"  # green
    elif score > 0.4:
        return "#f39c12"  # yellow
    else:
        return "#e74c3c"  # red


def score_to_rgba(score, alpha=0.8):
    """Map a [0,1] score to rgba color string."""
    if score > 0.7:
        return f"rgba(39,174,96,{alpha})"
    elif score > 0.4:
        return f"rgba(243,156,18,{alpha})"
    else:
        return f"rgba(231,76,60,{alpha})"


# ==========================================================
# Cylinder Mesh for Plotly
# ==========================================================

def make_cylinder_mesh(center, radius, height, color="#8B4513", opacity=0.8, n=20):
    """Generate a go.Mesh3d for a vertical cylinder.

    Parameters
    ----------
    center : array-like (3,)
        Centre of the cylinder [x, y, z].
    radius : float
        Cylinder radius in metres.
    height : float
        Cylinder height in metres.
    color : str
        Hex color.
    opacity : float
        Surface opacity.
    n : int
        Number of vertices around the circumference.

    Returns
    -------
    go.Mesh3d
    """
    center = np.asarray(center, dtype=float)
    theta = np.linspace(0, 2 * np.pi, n, endpoint=False)

    # Bottom circle
    xs_b = center[0] + radius * np.cos(theta)
    ys_b = center[1] + radius * np.sin(theta)
    zs_b = np.full(n, center[2] - height / 2)

    # Top circle
    xs_t = center[0] + radius * np.cos(theta)
    ys_t = center[1] + radius * np.sin(theta)
    zs_t = np.full(n, center[2] + height / 2)

    # Centre points (for caps)
    x_center_b, y_center_b, z_center_b = center[0], center[1], center[2] - height / 2
    x_center_t, y_center_t, z_center_t = center[0], center[1], center[2] + height / 2

    # Combine: [bottom ring (0..n-1), top ring (n..2n-1), bottom centre (2n), top centre (2n+1)]
    xs = np.concatenate([xs_b, xs_t, [x_center_b], [x_center_t]])
    ys = np.concatenate([ys_b, ys_t, [y_center_b], [y_center_t]])
    zs = np.concatenate([zs_b, zs_t, [z_center_b], [z_center_t]])

    ii, jj, kk = [], [], []
    bc = 2 * n  # bottom centre index
    tc = 2 * n + 1  # top centre index

    for i in range(n):
        ni = (i + 1) % n
        # Side faces (two triangles per quad)
        ii.extend([i, i])
        jj.extend([ni, i + n])
        kk.extend([i + n, ni + n])
        # Bottom cap
        ii.append(bc)
        jj.append(i)
        kk.append(ni)
        # Top cap
        ii.append(tc)
        jj.append(i + n)
        kk.append(ni + n)

    return go.Mesh3d(
        x=xs, y=ys, z=zs,
        i=ii, j=jj, k=kk,
        color=color,
        opacity=opacity,
        flatshading=True,
        name="cylinder",
        showlegend=False,
    )


# ==========================================================
# Arrow (candidate grasp pose) for Plotly
# ==========================================================

def make_arrow_trace(start, direction, length=0.05, color="#27ae60", width=4, name=""):
    """Generate a Scatter3d trace for an arrow (line + cone).

    Parameters
    ----------
    start : array-like (3,)
        Arrow start position.
    direction : array-like (3,)
        Arrow direction (will be normalized).
    length : float
        Arrow length in metres.
    color : str
        Arrow color.
    width : int
        Line width.
    name : str
        Trace name.

    Returns
    -------
    go.Scatter3d
    """
    start = np.asarray(start, dtype=float)
    direction = np.asarray(direction, dtype=float)
    norm = np.linalg.norm(direction)
    if norm > 0:
        direction = direction / norm
    end = start + direction * length

    return go.Scatter3d(
        x=[start[0], end[0]],
        y=[start[1], end[1]],
        z=[start[2], end[2]],
        mode="lines",
        line=dict(color=color, width=width),
        name=name,
        showlegend=False,
        hoverinfo="name",
    )


# ==========================================================
# Point Cloud Downsampling
# ==========================================================

def downsample_pcd(pcd, target_points=50000):
    """Downsample an Open3D point cloud to approximately target_points.

    Parameters
    ----------
    pcd : open3d.geometry.PointCloud
        Input point cloud.
    target_points : int
        Approximate desired number of points.

    Returns
    -------
    open3d.geometry.PointCloud
        Downsampled point cloud.
    """
    import open3d as o3d
    current = len(pcd.points)
    if current <= target_points:
        return pcd

    ratio = target_points / current
    voxel_size = 0.001 / (ratio ** 0.33)
    downsampled = pcd.voxel_down_sample(voxel_size=voxel_size)

    # If still too large, increase voxel size
    while len(downsampled.points) > target_points * 1.5 and voxel_size < 0.1:
        voxel_size *= 1.5
        downsampled = pcd.voxel_down_sample(voxel_size=voxel_size)

    return downsampled


# ==========================================================
# Quaternion → Direction Vector
# ==========================================================

def quat_to_direction(quat, axis="z"):
    """Convert quaternion (x,y,z,w) to a direction vector along the given axis.

    Parameters
    ----------
    quat : array-like (4,)
        Quaternion in (x, y, z, w) format.
    axis : str
        Which local axis to extract ('x', 'y', or 'z').

    Returns
    -------
    np.ndarray (3,)
        Direction vector in world frame.
    """
    from scipy.spatial.transform import Rotation
    R = Rotation.from_quat(quat).as_matrix()
    axis_map = {"x": 0, "y": 1, "z": 2}
    return R[:, axis_map[axis]]
