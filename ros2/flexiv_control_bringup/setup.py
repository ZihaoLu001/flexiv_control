from glob import glob

from setuptools import find_packages, setup

package_name = "flexiv_control_bringup"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Zihao Lu",
    maintainer_email="you@example.com",
    description=(
        "ROS 2 bringup for flexiv_control: a thin bridge node exposing the "
        "flexiv_control action contract over ROS 2 topics/services/actions. "
        "Community project, NOT affiliated with Flexiv Robotics."
    ),
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "control_node = flexiv_control_bringup.control_node:main",
        ],
    },
)
