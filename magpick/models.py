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
    """Cylindrical workpiece being picked by the magnetic gripper.

    All dimensional fields are in **meters** (SI convention, matching
    GraspGen / ROS / Open3D).  Use ``Billet.from_mm()`` to construct
    from human-friendly millimetre values.

    Attributes
    ----------
    id : int
        Unique billet identifier (from perception pipeline or manual entry).
    position : np.ndarray (3,)
        World-frame position of the billet centre (metres).
    orientation : np.ndarray (4,)
        World-frame orientation as quaternion (x, y, z, w).
    radius : float
        Cylinder radius in **metres**.  Sanity range: 0.001 – 1.0 m.
    length : float
        Cylinder length in **metres**.  Sanity range: 0.001 – 5.0 m.
    weight : float
        Mass in **kilograms**.
    material : str
        Key into the ``material_factor`` lookup table in config.
        Must be one of the recognised materials, or the evaluator
        will default to the worst-case factor (0.0).
    surface : str
        Key into the ``surface_factor`` lookup table in config.
    air_gap : float
        Additional user-specified air gap in **millimetres**
        (e.g., coating thickness, thermal clearance).  Defaults to
        0.0.  The curvature-induced gap for round billets is
        computed automatically by MagneticEvaluator.
    """

    id: int

    position: np.ndarray

    orientation: np.ndarray

    radius: float       # metres

    length: float       # metres

    weight: float       # kg

    material: str = "forged_steel"

    surface: str = "clean"

    air_gap: float = 0.0   # mm — additional gap beyond curvature

    def __post_init__(self):
        if not (0.001 <= self.radius <= 1.0):
            raise ValueError(
                f"Billet radius {self.radius} m is outside sanity range "
                f"(0.001 – 1.0 m).  Did you forget to convert from mm?  "
                f"Use Billet.from_mm() for millimetre input."
            )
        if not (0.001 <= self.length <= 5.0):
            raise ValueError(
                f"Billet length {self.length} m is outside sanity range "
                f"(0.001 – 5.0 m).  Did you forget to convert from mm?"
            )
        if self.weight <= 0:
            raise ValueError(
                f"Billet weight must be positive, got {self.weight} kg."
            )

    @classmethod
    def from_mm(
        cls,
        id: int,
        position: np.ndarray,
        orientation: np.ndarray,
        radius_mm: float,
        length_mm: float,
        weight_kg: float,
        material: str = "forged_steel",
        surface: str = "clean",
        air_gap_mm: float = 0.0,
    ) -> "Billet":
        """Construct a Billet from millimetre dimensions.

        This is the recommended constructor when entering dimensions
        by hand (e.g., from a drawing or operator input).
        """
        return cls(
            id=id,
            position=position,
            orientation=orientation,
            radius=radius_mm / 1000.0,
            length=length_mm / 1000.0,
            weight=weight_kg,
            material=material,
            surface=surface,
            air_gap=air_gap_mm,
        )

# ==========================================================
# Magnetic Gripper
# ==========================================================

@dataclass
class Gripper:
    """Magnetic gripper attached to the robot end-effector.

    All dimensional fields are in **metres** (SI).  Construct from
    a YAML profile via ``Gripper.from_profile()``.

    Attributes
    ----------
    name : str
        Human-readable gripper name (e.g., "Schmalz SGM-HP 40x121").
    max_force : float
        Rated holding force at zero air gap, clean forged steel, in **Newtons**.
    pad_width : float
        Short side of the rectangular pad footprint in **metres**.
        For a circular pad, this equals pad_length.
    pad_length : float
        Long side of the rectangular pad footprint in **metres**.
        For a circular pad, this equals pad_width.
    weight : float
        Gripper mass in **kilograms**.
    footprint_shape : str
        "rectangle" or "circle".
    pole_layout : PoleLayout or None
        Magnetic pole geometry, loaded from gripper profile YAML.
    """

    name: str
    max_force: float        # rated force at zero air gap, clean surface (N)
    pad_width: float         # meters — short side of footprint (== diameter if circular)
    pad_length: float        # meters — long side of footprint (== pad_width if circular)
    weight: float
    footprint_shape: str = "rectangle"
    pole_layout: object = None  # Optional[PoleLayout] — object to avoid import

    @classmethod
    def from_profile(cls, profile) -> "Gripper":
        w, l = profile.footprint.bounding_dims_m()
        return cls(
            name=profile.name,
            max_force=profile.rated_force_n,
            pad_width=w,
            pad_length=l,
            weight=profile.weight_kg,
            footprint_shape=profile.footprint.shape,
            pole_layout=getattr(profile, "pole_layout", None),
        )

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

    evaluator_results: List[EvaluationResult] = field(default_factory=list)

    final_score: float = 0.0

    rank: int = 0

    status: str = "NOT_EVALUATED"