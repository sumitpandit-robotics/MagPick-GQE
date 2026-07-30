import pytest
from magpick.gripper_profile import GripperProfile, list_available_profiles
from magpick.models import Gripper


PROFILE_PATH = "config/grippers/schmalz_sgm_hp_40x121.yaml"


def test_load_valid_profile():
    profile = GripperProfile.load(PROFILE_PATH)
    assert profile.name == "Schmalz SGM-HP 40x121"
    assert profile.footprint.shape == "rectangle"
    assert profile.footprint.width_m == pytest.approx(0.040)
    assert profile.footprint.length_m == pytest.approx(0.121)
    assert profile.tcp_depth_m == pytest.approx(0.1034)
    assert profile.rated_force_n == pytest.approx(1070.0)


def test_missing_file_raises():
    with pytest.raises(FileNotFoundError):
        GripperProfile.load("config/grippers/does_not_exist.yaml")


def test_missing_required_field_raises(tmp_path):
    bad_yaml = tmp_path / "bad.yaml"
    bad_yaml.write_text("name: 'Incomplete'\n")
    with pytest.raises(ValueError):
        GripperProfile.load(str(bad_yaml))


def test_gripper_from_profile_preserves_both_dimensions():
    profile = GripperProfile.load(PROFILE_PATH)
    gripper = Gripper.from_profile(profile)
    assert gripper.pad_width == pytest.approx(0.040)
    assert gripper.pad_length == pytest.approx(0.121)
    assert gripper.pad_width != gripper.pad_length  # would silently pass with old pad_diameter model