from setuptools import setup
import os
from glob import glob

package_name = "racecar_mission"

setup(
    name=package_name,
    version="2.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "config"), glob("config/*.json")),
        (os.path.join("share", package_name, "assets"), glob("assets/*.wav")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="racecar team",
    maintainer_email="team@example.com",
    description="穿越火线无人车任务程序",
    license="MIT",
    entry_points={
        "console_scripts": [
            "gateway = racecar_mission.gateway:main",
            "mission_node = racecar_mission.mission_node:main",
            "record_node = racecar_mission.record_node:main",
            "probe_topics = racecar_mission.probe_topics:main",
            "preflight = racecar_mission.preflight:main",
            "calibrate_drive = racecar_mission.calibrate_drive:main",
            "speak_test = racecar_mission.tts:main",
        ],
    },
)
