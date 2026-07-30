"""
base.py

Base evaluator interface.

Every evaluator in MagPick must inherit this class.
"""

from abc import ABC, abstractmethod

from magpick.models import (
    CandidatePose,
    Billet,
    Gripper,
    Scene,
    EvaluationResult,
)


class BaseEvaluator(ABC):

    @abstractmethod
    def evaluate(
        candidate,
        billet,
        gripper,
        scene,
        robot_motion=None,
    ) -> EvaluationResult:
        """
        Evaluate one grasp candidate.

        Returns
        -------
        EvaluationResult
        """
        pass