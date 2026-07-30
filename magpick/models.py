"""
models.py

Common data models used throughout the MagPick Grasp Quality Engine (GQE).

Author: Sumit Pandit
"""

from dataclasses import dataclass, field
from typing import Dict, List, Optional
import numpy as np


# ==========================================================
# Candidate Pose
# ==========================================================

@dataclass
class CandidatePose:
    """
    Represents one grasp candidate.
    """

    position: np.ndarray          # (3,)
    orientation: np.ndarray       # Quaternion (x,y,z,w)

    approach_vector: Optional[np.ndarray] = None
    surface_normal: Optional[np.ndarray] = None


# ==========================================================
# Billet Information
# ==========================================================

@dataclass
class Billet:

    id: int

    position: np.ndarray

    orientation: np.ndarray

    radius: float

    length: float

    weight: float

    material: str = "forged_steel"

    surface: str = "clean"
    air_gap: float = 0.0      # mm

# ==========================================================
# Magnetic Gripper
# ==========================================================

@dataclass
class Gripper:

    name: str

    max_force: float

    pad_diameter: float

    weight: float

@dataclass
class RobotMotion:

    velocity: float

    acceleration: float

    emergency_stop_acceleration: float = 0.0

# ==========================================================
# Scene Information
# ==========================================================

@dataclass
class Scene:

    point_cloud: object

    frame_id: str

    timestamp: float = 0.0


# ==========================================================
# Evaluation Result
# ==========================================================

@dataclass
class EvaluationResult:

    name: str

    passed: bool

    score: float

    weight: float

    reason: str

    details: Dict = field(default_factory=dict)


# ==========================================================
# Candidate Evaluation
# ==========================================================

@dataclass
class CandidateResult:

    candidate: CandidatePose

    evaluations: List[EvaluationResult] = field(default_factory=list)

    final_score: float = 0.0

    rank: int = 0

    status: str = "NOT_EVALUATED"