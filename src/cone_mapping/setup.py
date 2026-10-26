from glob import glob
import os

from setuptools import setup

package_name = "cone_mapping"

setup(
    name=package_name,
    version="1.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="driverless",
    maintainer_email="team@qutmotorsport.com",
    description="Observation-weighted cone map accumulation for absolute-pose sources",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "cone_mapper_node = cone_mapping.node_cone_mapper:main",
        ],
    },
)
