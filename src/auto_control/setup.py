import os
from glob import glob

from setuptools import find_packages, setup


package_name = "auto_control"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ohslo",
    maintainer_email="ohslo@example.com",
    description="Front-camera lane detection and autonomous-driving support.",
    license="TODO",
    entry_points={
        "console_scripts": [
            "front_lane_detector = auto_control.front_lane_detector:main",
            "front_lane_rotated_detector = auto_control.front_lane_rotated_detector:main",
            "lane_assist_drive = auto_control.lane_assist_drive:main",
        ],
    },
)
