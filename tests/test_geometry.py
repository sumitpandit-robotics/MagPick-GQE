import numpy as np

from magpick.models import *

from magpick.evaluators.geometry import GeometryEvaluator


candidate = CandidatePose(

    position=np.zeros(3),

    orientation=np.array([0,0,0,1])

)

billet = Billet(

    id=0,

    position=np.zeros(3),

    orientation=np.array([0,0,0,1]),

    radius=25,

    length=200,

    weight=2.5

)

gripper = Gripper(

    name="Schmalz",

    max_force=500,

    pad_diameter=40,

    weight=1.8

)

scene = Scene(

    point_cloud=None,

    frame_id="camera"

)

result = GeometryEvaluator().evaluate(

    candidate,

    billet,

    gripper,

    scene,

)

print()

print(result)

print()

print(result.details)