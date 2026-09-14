from setuptools import find_packages, setup
import os
from glob import glob

package_name = "gazebo_nav2_integrations"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(include=[package_name, package_name + ".*"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="devrobot",
    maintainer_email="peeravatanachartnol@firstloop-tech.com",
    description="Mirrors Nav2 map/pose state into Gazebo Classic",
    license="MIT",
    entry_points={
        "console_scripts": [
            "map_to_gazebo = gazebo_nav2_integrations.map_to_gazebo.node:main",
        ],
    },
)
