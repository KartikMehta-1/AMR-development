from setuptools import setup


package_name = "amr_safety"

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
    description="AMR passive safety supervisor and safety diagnostics.",
    license="Proprietary",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "safety_supervisor = amr_safety.safety_supervisor:main",
        ],
    },
)
