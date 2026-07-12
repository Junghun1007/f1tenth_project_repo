from setuptools import find_packages, setup


package_name = "manual_control"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ohslo",
    maintainer_email="ohslo@example.com",
    description="Manual controller input translator for AutoDrive.",
    license="TODO",
    entry_points={
        "console_scripts": [
            "manual_control_node = manual_control.manual_control_node:main",
            "manual_to_vesc_node = manual_control.manual_to_vesc_node:main",
        ],
    },
)
