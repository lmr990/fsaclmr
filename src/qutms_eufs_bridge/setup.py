from glob import glob
from setuptools import setup


package_name = "qutms_eufs_bridge"


setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="QUT Motorsport",
    maintainer_email="team@qutmotorsport.com",
    description="Launch-only QUTMS to eufs_sim control-loop integration.",
    license="MIT",
)
