from glob import glob
import os
from setuptools import setup

package_name = "qutms_test_gui"

setup(
    name=package_name,
    version="0.1.0",
    packages=[
        package_name,
        package_name + ".gui",
        package_name + ".ros",
        package_name + ".process",
    ],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools", "PySide6"],
    zip_safe=True,
    maintainer="QUT Motorsport",
    maintainer_email="team@qutmotorsport.com",
    description="QUTMS Driverless x EUFS Simulator test and control GUI.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "qutms_test_gui = " + package_name + ".main:main",
        ],
    },
)
