from glob import glob
import os

from setuptools import find_packages, setup


package_name = "tunnel_data_collection"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml", "README.md"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ohslo",
    maintainer_email="ohslo@example.com",
    description="Tunnel RGB/BEV data capture and extraction.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "extract_frames = tunnel_data_collection.extract_frames:main",
        ],
    },
)
