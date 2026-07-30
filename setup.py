from setuptools import setup, find_packages

setup(
    name="magpick-gqe",
    version="1.1.0",
    description="Industrial Grasp Quality Evaluation Framework for Magnetic Grippers",
    long_description=open("README.md").read(),
    long_description_content_type="text/markdown",
    author="Sumit Pandit",
    author_email="sumit.pandit@robotics.dev",
    url="https://github.com/sumitpandit-robotics/MagPick-GQE",
    python_requires=">=3.10",
    packages=find_packages(),
    install_requires=[
        "numpy>=1.21",
        "open3d>=0.16",
        "scipy>=1.7",
        "pyyaml>=6.0",
    ],
    classifiers=[
        "Development Status :: 4 - Beta",
        "Intended Audience :: Science/Research",
        "Topic :: Scientific/Engineering :: Artificial Intelligence",
        "Programming Language :: Python :: 3.11",
    ],
)