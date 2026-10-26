from glob import glob
from os.path import join

from setuptools import find_packages, setup

package_name = "webcam_path_demo"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        (join("share", package_name), ["package.xml"]),
        (join("share", package_name, "config"), glob("config/*.yaml")),
        (join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="QUTMS Demo",
    description="Webcam cone detection + path overlay demo",
    license="MIT",
    entry_points={
        "console_scripts": [
            "webcam_path_node = webcam_path_demo.node_webcam_path:main",
            "webcam_calibrate = webcam_path_demo.calibrate_webcam:main",
            "mock_velocity = webcam_path_demo.mock_velocity:main",
        ],
    },
)
