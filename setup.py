"""Setup script for PyBullet ROS2 package."""

import glob

from setuptools import setup

package_name = "pybullet_ros2"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            [f"resource/{package_name}"],
        ),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob.glob("launch/*.launch.py")),
        (f"share/{package_name}/config", glob.glob("config/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Jafar Uruç",
    maintainer_email="jafar.uruc@gmail.com",
    description="A ROS2 interface to PyBullet",
    license="BSD-3-Clause",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "pybullet_ros2_node = pybullet_ros2.pybullet_ros2_node:main",
        ],
    },
)
