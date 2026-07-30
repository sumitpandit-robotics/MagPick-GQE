from magpick.evaluators.contact_area import ContactAreaEvaluator
from magpick.models import *

evaluator = ContactAreaEvaluator()

gripper = Gripper(
    name="Schmalz",
    max_force=560,
    pad_diameter=40,
    weight=2.8,
)

billet = Billet(
    id=1,
    position=None,
    orientation=None,
    radius=20,
    length=250,
    weight=2.5,
)

metrics = evaluator.compute_metrics(
    None,
    billet,
    gripper,
)

print(metrics)