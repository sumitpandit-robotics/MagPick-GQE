"""
test_utils.py

Tests for point cloud and geometry utility functions.
"""

import numpy as np
import open3d as o3d
import pytest

from magpick.utils.geometry import (
    load_point_cloud,
    estimate_normals,
    crop_sphere,
    nearest_point,
    compute_normal,
)


@pytest.fixture
def pcd():
    return load_point_cloud("datasets/scene.ply")


def test_load_point_cloud(pcd):
    assert not pcd.is_empty()
    assert len(pcd.points) > 0


def test_point_cloud_has_xyz(pcd):
    points = np.asarray(pcd.points)
    assert points.shape[1] == 3


def test_estimate_normals(pcd):
    estimate_normals(pcd, radius=0.02)
    assert len(pcd.normals) == len(pcd.points)


def test_crop_sphere_reduces_size(pcd):
    center = np.asarray(pcd.points)[0]
    cropped = crop_sphere(pcd, center, radius=0.04)
    assert len(cropped.points) > 0
    assert len(cropped.points) <= len(pcd.points)


def test_crop_sphere_outside_returns_empty(pcd):
    """Cropping far from the point cloud should return nothing."""
    far_away = np.array([1000.0, 1000.0, 1000.0])
    cropped = crop_sphere(pcd, far_away, radius=0.04)
    assert len(cropped.points) == 0


def test_nearest_point_returns_valid_index(pcd):
    center = np.asarray(pcd.points)[0]
    idx = nearest_point(pcd, center)
    assert 0 <= idx < len(pcd.points)


def test_nearest_point_of_center_is_close(pcd):
    """The nearest point to the centroid should be near the centroid."""
    points = np.asarray(pcd.points)
    centroid = points.mean(axis=0)
    idx = nearest_point(pcd, centroid)
    dist = np.linalg.norm(points[idx] - centroid)
    assert dist < 0.5  # within 500mm


def test_compute_normal_returns_unit_vector(pcd):
    estimate_normals(pcd, radius=0.02)
    normal = compute_normal(pcd, 0)
    norm = np.linalg.norm(normal)
    assert norm == pytest.approx(1.0, abs=1e-3)


def test_compute_normal_out_of_range_raises(pcd):
    estimate_normals(pcd, radius=0.02)
    with pytest.raises(IndexError):
        compute_normal(pcd, -1)
    with pytest.raises(IndexError):
        compute_normal(pcd, len(pcd.points))
