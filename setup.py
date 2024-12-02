import os

from setup_utils import include_directory
from setuptools import find_packages, setup

package_name = "simulation"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        *include_directory(
            install_path=os.path.join("share", package_name, "config"),
            source_path="config",
        ),
        *include_directory(
            install_path=os.path.join("share", package_name, "launch"),
            source_path="launch",
        ),
        *include_directory(
            install_path=os.path.join("share", package_name, "models"),
            source_path="models",
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="DHBW Smart Rollerz",
    maintainer_email="info@dhbw-smartrollerz.org",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            f"robot_description_publisher = {package_name}.robot_description_publisher:main",
            f"car_control_bridge = {package_name}.car_control_bridge:main",
        ],
    },
)
