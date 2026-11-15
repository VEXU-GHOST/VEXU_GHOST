import os
from glob import glob

from setuptools import find_packages, setup

package_name = "ghost_cv_sim"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "worlds"), glob("worlds/*.world")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "config", "scenarios"), glob("config/scenarios/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="GHOST",
    maintainer_email="ghost-cv@example.com",
    description="Gazebo test bench for the CV model",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "scene_manager = ghost_cv_sim.scene_manager_node:main",
            "camera_scanner = ghost_cv_sim.camera_scanner_node:main",
            "cv_detector = ghost_cv_sim.cv_detector_node:main",
            "cv_evaluator = ghost_cv_sim.evaluator_node:main",
        ],
    },
)
