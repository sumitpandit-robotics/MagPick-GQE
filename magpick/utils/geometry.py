"""
geometry.py

Geometry utilities for MagPick.

All evaluators use these functions.

Author : Sumit Pandit
"""

import copy
import numpy as np
import open3d as o3d

from scipy.spatial.transform import Rotation
# ==========================================================
# Load Point Cloud
# ==========================================================

def load_point_cloud(filename: str):

    pcd = o3d.io.read_point_cloud(filename)

    if pcd.is_empty():
        raise RuntimeError(f"Unable to load {filename}")

    return pcd


# ==========================================================
# Estimate Surface Normals
# ==========================================================

def estimate_normals(
    pcd,
    radius=0.02,
    max_nn=30,
):

    pcd.estimate_normals(

        search_param=o3d.geometry.KDTreeSearchParamHybrid(

            radius=radius,

            max_nn=max_nn,

        )

    )

    return pcd


# ==========================================================
# Crop Local Region
# ==========================================================

def crop_sphere(
    pcd,
    center,
    radius,
):

    points = np.asarray(pcd.points)

    dist = np.linalg.norm(points - center, axis=1)

    idx = np.where(dist <= radius)[0]

    return pcd.select_by_index(idx)


# ==========================================================
# Nearest Point
# ==========================================================

def nearest_point(
    pcd,
    query,
):

    tree = o3d.geometry.KDTreeFlann(pcd)

    _, idx, _ = tree.search_knn_vector_3d(query, 1)

    return idx[0]


# ==========================================================
# Surface Normal
# ==========================================================
# ==========================================================
# Surface Normal
# ==========================================================

def compute_normal(
    pcd,
    index,
):
    """
    Returns the surface normal at the specified point index.

    If the point cloud has no normals, they are estimated automatically.
    """

    # Compute normals if missing
    if len(pcd.normals) == 0:

        pcd.estimate_normals(

            search_param=o3d.geometry.KDTreeSearchParamHybrid(

                radius=0.02,
                max_nn=30,

            )

        )

        pcd.normalize_normals()

    normals = np.asarray(pcd.normals)

    # Safety check
    if index < 0 or index >= len(normals):

        raise IndexError(
            f"Point index {index} out of range "
            f"(normals available: {len(normals)})"
        )

    return normals[index]


# ==========================================================
# Transform Point
# ==========================================================

def transform_point(
    T,
    point,
):

    p = np.append(point, 1.0)

    p = T @ p

    return p[:3]
# ==========================================================
# Angle Between Two Vectors
# ==========================================================

def angle_between(v1, v2):
    """
    Returns the angle (degrees) between two 3D vectors.
    """

    v1 = np.asarray(v1, dtype=float)
    v2 = np.asarray(v2, dtype=float)

    v1 = v1 / np.linalg.norm(v1)
    v2 = v2 / np.linalg.norm(v2)

    dot = np.clip(np.dot(v1, v2), -1.0, 1.0)

    return np.degrees(np.arccos(dot))
# ==========================================================
# Quaternion → Approach Vector
# ==========================================================

def quaternion_to_approach(quaternion):
    """
    Convert quaternion (x,y,z,w) into the tool approach vector.

    Assumes the gripper approaches along its local +Z axis.

    Returns
    -------
    np.ndarray
        Unit approach vector in world coordinates.
    """

    rotation = Rotation.from_quat(quaternion)

    rotation_matrix = rotation.as_matrix()

    approach = rotation_matrix[:, 2]

    approach /= np.linalg.norm(approach)

    return approach