import os
from glob import glob

from setuptools import setup

package_name = "p3dx_base"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="jcp",
    maintainer_email="jcp@todo.todo",
    description="Pioneer 3DX base driver (ARCOS serial protocol) for ROS 2",
    license="MIT",
    entry_points={
        "console_scripts": [
            "p3dx_base_node = p3dx_base.p3dx_base_node:main",
        ],
    },
)
