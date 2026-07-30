"""
parser.py

Converts grasp planner output into
MagPick CandidatePose objects.
"""

from typing import List

from magpick.models import CandidatePose


class GraspParser:

    def parse(
        self,
        planner_output,
    ) -> List[CandidatePose]:
        """
        Placeholder parser.

        Later this will convert:
            GraspGen
            GraspNet
            AnyGrasp
            etc.

        into CandidatePose objects.
        """

        return []