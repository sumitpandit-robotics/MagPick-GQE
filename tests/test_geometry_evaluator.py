import numpy as np
import open3d as o3d

from magpick.models import (
    CandidatePose,
    Billet,
    Gripper,
    Scene,
)

from magpick.evaluators.geometry import GeometryEvaluator

from magpick.utils.geometry import estimate_normals


# ==========================================================
# Load Scene
# ==========================================================

pcd = o3d.io.read_point_cloud("datasets/scene.ply")

estimate_normals(
    pcd,
    radius=0.02,
)

scene = Scene(
    point_cloud=pcd,
    frame_id="world",
)

# ==========================================================
# Dummy Billet
# ==========================================================

billet = Billet(
    diameter=0.05,
    length=0.30,
    weight=3.0,
)

# ==========================================================
# Dummy Gripper
# ==========================================================

gripper = Gripper(
    name="Schmalz",
)

# ==========================================================
# Candidate
# ==========================================================

points = np.asarray(pcd.points)

candidate = CandidatePose(
    position=points[0],
    orientation=np.array([0, 0, 0, 1]),
)

# ==========================================================
# Evaluate
# ==========================================================

result = GeometryEvaluator().evaluate(
    candidate,
    billet,
    gripper,
    scene,
)

print()

print("=" * 60)

print(result)

print("=" * 60)