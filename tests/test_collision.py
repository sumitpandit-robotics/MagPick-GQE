import open3d as o3d
import numpy as np

from magpick.models import *
from magpick.evaluators.collision import CollisionEvaluator


pcd = o3d.io.read_point_cloud(
    "datasets/scene.ply"
)

scene = Scene(
    point_cloud=pcd,
    frame_id="world",
)

candidate = CandidatePose(
    position=np.array([0.0, 0.0, 0.0]),
    orientation=np.array([0.0, 0.0, 0.0, 1.0]),
)

evaluator = CollisionEvaluator()

metrics = evaluator.compute_metrics(
    candidate,
    scene,
)

print(metrics)