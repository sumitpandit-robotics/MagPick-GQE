"""
fusion.py

MagPick-GQE

Fusion Engine

Combines all evaluator scores and ranks grasp candidates.
"""

from magpick.models import CandidateResult
from magpick.evaluators.geometry import GeometryEvaluator
from magpick.evaluators.magnetic import MagneticEvaluator
from magpick.evaluators.contact_area import ContactAreaEvaluator
from magpick.evaluators.collision import CollisionEvaluator
from magpick.evaluators.pole_coverage import PoleCoverageEvaluator
from magpick.evaluators.robot_dynamics import RobotDynamicsEvaluator


class FusionEngine:

    def __init__(self):

        self.evaluators = [

            GeometryEvaluator(),

            ContactAreaEvaluator(),

            MagneticEvaluator(),

            PoleCoverageEvaluator(),

            CollisionEvaluator(),

            RobotDynamicsEvaluator(),
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

        all_passed = True

        for evaluator in self.evaluators:

            result = evaluator.evaluate(

                candidate=candidate,

                billet=billet,

                gripper=gripper,

                scene=scene,

                robot_motion=robot_motion,

            )

            evaluator_results.append(result)

            if not result.passed:

                all_passed = False

            weighted_sum += result.score * result.weight

            total_weight += result.weight

        if total_weight == 0:

            final_score = 0.0

        else:

            final_score = weighted_sum / total_weight

        # Hard-constraint enforcement: if any evaluator failed its
        # minimum threshold, the candidate is disqualified regardless
        # of its weighted average score.  This prevents a candidate
        # that violates a safety rule (e.g., insufficient holding
        # force) from being selected because other scores are high.
        if not all_passed:

            final_score = 0.0

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

        for i, result in enumerate(results):

            result.rank = i + 1

            result.status = (
                "PASS" if result.final_score > 0.0 else "FAIL"
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