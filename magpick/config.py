"""
config.py

Loads MagPick configuration.
"""

from pathlib import Path
import yaml


class Config:

    def __init__(self):

        config_file = (
            Path(__file__).parent.parent
            / "config"
            / "weights.yaml"
        )

        with open(config_file, "r") as f:
            self.data = yaml.safe_load(f)

    def __getitem__(self, key):
        return self.data[key]


config = Config()