"""
fusion.py

MagPick-GQE

Fusion Engine

Combines all evaluator scores and ranks grasp candidates.
"""

from dataclasses import dataclass

from magpick.evaluators.geometry import GeometryEvaluator
from magpick.evaluators.magnetic import MagneticEvaluator
from magpick.evaluators.contact_area import ContactAreaEvaluator
from magpick.evaluators.collision import CollisionEvaluator


@dataclass
class CandidateResult:

    candidate: object

    final_score: float

    evaluator_results: list


class FusionEngine:

    def __init__(self):

        self.evaluators = [

            GeometryEvaluator(),

            ContactAreaEvaluator(),

            MagneticEvaluator(),

            CollisionEvaluator(),
        ]

    def evaluate_candidate(

        self,

        candidate,

        billet,

        gripper,

        scene,

        robot_motion=None,

    ):

        evaluator_results = []

        weighted_sum = 0.0

        total_weight = 0.0

        for evaluator in self.evaluators:

            result = evaluator.evaluate(

                candidate=candidate,

                billet=billet,

                gripper=gripper,

                scene=scene,

                robot_motion=robot_motion,

            )

            evaluator_results.append(result)

            weighted_sum += result.score * result.weight

            total_weight += result.weight

        if total_weight == 0:

            final_score = 0.0

        else:

            final_score = weighted_sum / total_weight

        return CandidateResult(

            candidate=candidate,

            final_score=final_score,

            evaluator_results=evaluator_results,

        )

    def rank_candidates(

        self,

        candidates,

        billet,

        gripper,

        scene,

        robot_motion=None,

    ):

        results = []

        for candidate in candidates:

            result = self.evaluate_candidate(

                candidate,

                billet,

                gripper,

                scene,

                robot_motion,

            )

            results.append(result)

        results.sort(

            key=lambda r: r.final_score,

            reverse=True,

        )

        return results

    def best_candidate(

        self,

        candidates,

        billet,

        gripper,

        scene,

        robot_motion=None,

    ):

        ranked = self.rank_candidates(

            candidates,

            billet,

            gripper,

            scene,

            robot_motion,

        )

        if len(ranked) == 0:

            return None

        return ranked[0]