"""
interface.py

Defines the interface between any AI grasp planner
(GraspGen, GraspNet, AnyGrasp, etc.)
and MagPick-GQE.
"""

from abc import ABC, abstractmethod
from typing import List

from magpick.models import (
    Scene,
    Billet,
    CandidatePose,
)


class GraspProvider(ABC):
    """
    Base interface for all grasp planners.
    """

    @abstractmethod
    def generate_candidates(
        self,
        scene: Scene,
        billet: Billet,
    ) -> List[CandidatePose]:
        """
        Generate grasp candidates for one billet.

        Returns
        -------
        List[CandidatePose]
        """
        pass