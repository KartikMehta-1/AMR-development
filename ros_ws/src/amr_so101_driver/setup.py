from glob import glob

from setuptools import setup


package_name = "amr_so101_driver"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Kartik",
    maintainer_email="kartik@example.com",
    description="Conservative SO-101 FollowJointTrajectory bridge for MoveIt bring-up.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "joint_state_merger = amr_so101_driver.joint_state_merger:main",
            "so101_trajectory_bridge = amr_so101_driver.so101_trajectory_bridge:main",
        ],
    },
)
