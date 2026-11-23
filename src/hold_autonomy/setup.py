import os
from glob import glob
from setuptools import find_packages, setup

package_name = "hold_autonomy"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "resource"), glob("resource/*.urdf")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Hanna Pyrhonen",
    maintainer_email="student@example.com",
    description="TEKS4430 project: autonomous cargo hold gas monitoring and ventilation",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "plant_sim = hold_autonomy.plant_sim:main",
            "perception = hold_autonomy.perception:main",
            "hold_manager = hold_autonomy.hold_manager:main",
            "vent_control = hold_autonomy.vent_control:main",
            "rover_control = hold_autonomy.rover_control:main",
            "recorder = hold_autonomy.recorder:main",
        ],
    },
)
