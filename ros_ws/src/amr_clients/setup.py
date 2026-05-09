from setuptools import setup


package_name = "amr_clients"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Kartik",
    maintainer_email="kartik@example.com",
    description="Shared ROS client helpers for AMR mission, safety, navigation, localization, and STM diagnostics.",
    license="Proprietary",
    tests_require=["pytest"],
)
