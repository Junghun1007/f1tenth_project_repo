from glob import glob
import os

from setuptools import find_packages, setup


package_name = "image_processor"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ohslo",
    maintainer_email="ohslo@example.com",
    description="Normal image passthrough publisher for AutoDrive.",
    license="TODO",
    entry_points={
        "console_scripts": [
            "normal_image_publisher_node = "
            "image_processor.normal_image_publisher_node:main",
        ],
    },
)
