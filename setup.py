import os
from glob import glob

from setuptools import find_packages, setup

package_name = "ipb_car_monitor"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/" + package_name, ["package.xml"]),
        # Install config files
        (
            os.path.join("share", package_name, "config"),
            glob(os.path.join("config", "*.yaml")),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Benedikt Mersch",
    maintainer_email="mersch@igg.uni-bonn.de",
    description="Main ipb-car monitor",
    license="MIT",
    entry_points={
        "console_scripts": ["monitor = ipb_car_monitor.monitor:main"],
    },
)
