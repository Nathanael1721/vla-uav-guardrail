from setuptools import find_packages, setup

package_name = "safety_shield_node"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Kuan-Ting Lai",
    maintainer_email="kuantinglai@gmail.com",
    description="Safety Shield ROS 2 node (Humble) wrapping the pure core.",
    license="Proprietary",
    entry_points={
        "console_scripts": [
            "shield_node = safety_shield_node.node:main",
        ],
    },
)
