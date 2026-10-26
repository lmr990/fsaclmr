from glob import glob
import os

from setuptools import setup

package_name = "yolo_cone_detection"

setup(
    name=package_name,
    version="1.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="QUTMS",
    maintainer_email="team@qutmotorsport.com",
    description="YOLO-based cone detection node for FSAC autonomous racecar (TCone-YOLO).",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "yolo_detector_node = yolo_cone_detection.node_yolo_detector:main",
        ],
    },
)
