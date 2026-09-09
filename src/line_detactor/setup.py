from glob import glob
import os

from setuptools import find_packages, setup


package_name = "line_detactor"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            [f"resource/{package_name}"],
        ),
        (f"share/{package_name}", ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "models"), glob("models/*.pt")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ohslo",
    maintainer_email="ohslo@example.com",
    description="GPU Fast-SCNN preview for raw BEV lane segmentation.",
    license="TODO",
    entry_points={
        "console_scripts": [
            "line_detactor_node = line_detactor.line_detactor_node:main",
        ],
    },
)
