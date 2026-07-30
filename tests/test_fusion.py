import numpy as np
import open3d as o3d

from magpick.models import *

from magpick.fusion import FusionEngine


pcd = o3d.io.read_point_cloud(
    "datasets/scene.ply"
)

scene = Scene(

    point_cloud=pcd,

    frame_id="world",
)

billet = Billet(

    id=1,

    position=np.array([0.0, 0.0, 0.0]),

    orientation=np.array([0.0, 0.0, 0.0, 1.0]),

    radius=20,

    length=200,

    weight=2.5,
)

gripper = Gripper(

    name="SGM HP",

    max_force=560,

    pad_diameter=40,

    weight=1.2,
)

robot = RobotMotion(

    velocity=1500,

    acceleration=3000,
)

candidates = [

    CandidatePose(

        position=np.array([0.0, 0.0, 0.0]),

        orientation=np.array([0.0, 0.0, 0.0, 1.0]),

    ),

    CandidatePose(

        position=np.array([0.02, 0.0, 0.0]),

        orientation=np.array([0.0, 0.0, 0.0, 1.0]),

    ),

]

engine = FusionEngine()

ranked = engine.rank_candidates(

    candidates,

    billet,

    gripper,

    scene,

    robot,
)

print()

print("=" * 60)

print("MAGPICK GQE")

print("=" * 60)

for i, result in enumerate(ranked):

    print()

    print(f"Rank {i+1}")

    print(f"Score : {result.final_score:.3f}")

    for r in result.evaluator_results:

        print(

            f"{r.name:15}"

            f"{r.score:.3f}"

        )

print()

print("=" * 60)

print("BEST")

print("=" * 60)

best = engine.best_candidate(

    candidates,

    billet,

    gripper,

    scene,

    robot,
)

print(best.final_score)