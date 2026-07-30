"""
gripper_profile.py

Generic loader for magnetic gripper profiles.

Adding support for a new gripper requires ONLY a new YAML file under
config/grippers/ — no Python changes should be needed for a gripper whose
footprint fits "rectangle" or "circle".

Author: Sumit Pandit
"""

from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional
import yaml


SUPPORTED_FOOTPRINT_SHAPES = {"rectangle", "circle"}


@dataclass
class GripperFootprint:
    shape: str              # "rectangle" | "circle"
    width_m: float = 0.0    # rectangle: short side. circle: unused.
    length_m: float = 0.0   # rectangle: long side. circle: unused.
    diameter_m: float = 0.0  # circle only.

    def bounding_dims_m(self):
        """(min_dim, max_dim) envelope — used by evaluators that need a
        shape-agnostic size comparison regardless of footprint type."""
        if self.shape == "rectangle":
            return (min(self.width_m, self.length_m), max(self.width_m, self.length_m))
        if self.shape == "circle":
            return (self.diameter_m, self.diameter_m)
        raise ValueError(f"Unsupported footprint shape: {self.shape}")


@dataclass
class PoleLayout:
    """Magnetic pole geometry for a gripper.

    Attributes
    ----------
    num_poles : int
        Number of magnetic poles on the gripping face.
    pole_positions_m : list of (x, y) tuples
        Centres of each pole in the gripper's local frame (metres).
    pole_diameter_m : float
        Diameter of each circular pole face (metres).
    """
    num_poles: int = 0
    pole_positions_m: List[tuple] = field(default_factory=list)
    pole_diameter_m: float = 0.0


@dataclass
class GripperProfile:
    name: str
    type: str
    footprint: GripperFootprint
    tcp_depth_m: float
    rated_force_n: float
    weight_kg: float
    mesh_path: Optional[str] = None

    # MRD 5.2: force derating curves (holding force vs air gap in mm)
    force_curve: Dict[float, float] = field(default_factory=dict)

    # MRD 5.2: centre of gravity in gripper local frame (metres)
    cog_m: Optional[tuple] = None

    # MRD 5.2: moment of inertia about each axis (kg·m²)
    inertia_kgm2: Optional[Dict[str, float]] = None

    # MRD 5.2: magnetic pole layout
    pole_layout: Optional[PoleLayout] = None

    @classmethod
    def load(cls, yaml_path: str) -> "GripperProfile":
        path = Path(yaml_path)
        if not path.exists():
            raise FileNotFoundError(
                f"Gripper profile not found: {yaml_path}\n"
                f"Available profiles: {list_available_profiles()}"
            )
        with open(path, "r") as f:
            data = yaml.safe_load(f)

        cls._validate(data, yaml_path)

        fp_data = data["footprint"]
        shape = fp_data["shape"]
        if shape not in SUPPORTED_FOOTPRINT_SHAPES:
            raise ValueError(
                f"Unsupported footprint shape '{shape}' in {yaml_path}. "
                f"Supported: {sorted(SUPPORTED_FOOTPRINT_SHAPES)}"
            )
        footprint = GripperFootprint(
            shape=shape,
            width_m=fp_data.get("width_m", 0.0),
            length_m=fp_data.get("length_m", 0.0),
            diameter_m=fp_data.get("diameter_m", 0.0),
        )

        # Force curve: {air_gap_mm: holding_force_N}
        force_data = data.get("force", {})
        force_curve_raw = force_data.get("force_curve", {})
        force_curve = {float(k): float(v) for k, v in force_curve_raw.items()}

        # Centre of gravity
        cog = data.get("cog_m")
        if isinstance(cog, (list, tuple)) and len(cog) == 3:
            cog = tuple(cog)
        else:
            cog = None

        # Inertia
        inertia = data.get("inertia_kgm2")

        # Pole layout
        pole_data = data.get("pole_layout")
        pole_layout = None
        if pole_data and isinstance(pole_data, dict):
            pole_layout = PoleLayout(
                num_poles=pole_data.get("num_poles", 0),
                pole_positions_m=[
                    tuple(p) for p in pole_data.get("pole_positions_m", [])
                ],
                pole_diameter_m=pole_data.get("pole_diameter_m", 0.0),
            )

        return cls(
            name=data["name"],
            type=data.get("type", "magnetic"),
            footprint=footprint,
            tcp_depth_m=data["tcp"]["depth_m"],
            rated_force_n=force_data.get("rated_force_n", 0.0),
            weight_kg=data.get("weight_kg", 0.0),
            mesh_path=data.get("mesh"),
            force_curve=force_curve,
            cog_m=cog,
            inertia_kgm2=inertia,
            pole_layout=pole_layout,
        )

    @staticmethod
    def _validate(data: dict, source: str):
        for key in ("name", "footprint", "tcp", "force"):
            if key not in data:
                raise ValueError(f"Gripper profile {source} missing required field: '{key}'")
        if "depth_m" not in data["tcp"]:
            raise ValueError(f"Gripper profile {source}: tcp.depth_m is required")
        if "rated_force_n" not in data["force"]:
            raise ValueError(f"Gripper profile {source}: force.rated_force_n is required")
        if "shape" not in data["footprint"]:
            raise ValueError(f"Gripper profile {source}: footprint.shape is required")


def list_available_profiles(config_dir: str = "config/grippers") -> list:
    p = Path(config_dir)
    if not p.exists():
        return []
    return sorted(f.stem for f in p.glob("*.yaml"))