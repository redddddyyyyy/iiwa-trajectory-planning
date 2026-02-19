from setuptools import find_packages, setup

package_name = "iiwa_trajectory_demo"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", ["launch/iiwa_replay.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Rajeev Reddy",
    maintainer_email="rajeevreddy1009@gmail.com",
    description="Replay MATLAB-generated iiwa trajectories to a ROS2 JointTrajectory controller for Gazebo demos.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "trajectory_replay = iiwa_trajectory_demo.trajectory_replay:main",
        ],
    },
)
