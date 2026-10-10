import os
from glob import glob

from setuptools import setup


package_name = "orange_lane_perception"


setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Orange IGVC",
    maintainer_email="todo@example.com",
    description="Lane-perception baseline and metric recording tools for Orange IGVC.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "lane_detector = orange_lane_perception.lane_detector:main",
            "lane_prediction_recorder = orange_lane_perception.lane_prediction_recorder:main",
        ],
    },
)
