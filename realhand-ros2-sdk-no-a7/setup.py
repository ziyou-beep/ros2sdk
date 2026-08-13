from glob import glob

from setuptools import find_packages, setup

package_name = "realhand_ros2"


setup(
    name=package_name,
    version="0.1.0",
    package_dir={"": "src"},
    packages=find_packages(where="src"),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (
            f"share/{package_name}/examples",
            ["examples/README.md"] + glob("examples/*.py") + glob("examples/*.sh"),
        ),
    ],
    install_requires=["setuptools", "realhand", "PyQt5"],
    zip_safe=True,
    maintainer="realhand",
    maintainer_email="realhand@todo.todo",
    description="ROS2 bridge package for the RealHand Python SDK",
    license="MIT",
    entry_points={
        "console_scripts": [
            "realhand_hand_node = realhand_ros2.hand.node:main",
            "realhand_arm_node = realhand_ros2.arm.node:main",
            "realhand_gui = realhand_ros2.gui.app:main",
        ],
    },
)
