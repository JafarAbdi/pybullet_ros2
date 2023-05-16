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
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="juruc",
    maintainer_email="jafar.uruc@gmail.com",
    description="A ROS2 interface to PyBullet",
    license="BSD",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [],
    },
)
