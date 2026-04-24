from glob import glob
from setuptools import find_packages, setup

package_name = "room_vision"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
            ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/models", glob("models/*.onnx")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Philip Tice",
    maintainer_email="tice.philip.t@gmail.com",
    description="YOLOv8-based object detection node for the BRT7K robot.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "detector = room_vision.detector_node:main",
        ],
    },
)
