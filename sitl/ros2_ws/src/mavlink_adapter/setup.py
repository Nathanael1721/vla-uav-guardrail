from glob import glob

from setuptools import find_packages, setup

package_name = "mavlink_adapter"

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
    description="Shield -> MAVROS 2 bridge; the single body -> local-NED boundary.",
    license="Proprietary",
    entry_points={
        "console_scripts": [
            "mavlink_adapter = mavlink_adapter.adapter_node:main",
        ],
    },
)
