import os
from glob import glob

from setuptools import setup

package_name = "p3dx_task"

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
    description="Task orchestration (patrol) for Pioneer 3DX",
    license="MIT",
    entry_points={
        "console_scripts": [
            "patrol_node = p3dx_task.patrol_node:main",
        ],
    },
)
