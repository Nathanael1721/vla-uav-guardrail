from glob import glob

from setuptools import find_packages, setup

package_name = "safety_shield"

setup(
    name=package_name,
    version="0.2.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Nathanael Tjahyadi",
    maintainer_email="nathanaelcayadi@gmail.com",
    description="Guardrail Safety Shield ROS 2 node, VLA stub node and rail launch file.",
    license="Proprietary",
    entry_points={
        "console_scripts": [
            "shield_node = safety_shield.shield_node:main",
            "vla_stub_node = safety_shield.vla_stub_node:main",
        ],
    },
)
