from setuptools import setup, find_packages

setup(
    name="magpick",
    version="0.1.0",
    packages=find_packages(),
    install_requires=[
        "numpy",
        "open3d",
        "scipy",
        "pyyaml",
    ],
)