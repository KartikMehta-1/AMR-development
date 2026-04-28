from setuptools import setup

package_name = "amr_missions"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config", ["config/places.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="kartik",
    maintainer_email="kartik@example.com",
    description="Named-place mission layer on top of Nav2 for the AMR.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "mission_cli = amr_missions.mission_cli:main",
            "mission_server = amr_missions.mission_server:main",
        ],
    },
)
