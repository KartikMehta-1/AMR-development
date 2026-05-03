from setuptools import setup


package_name = "amr_voice"

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
    description="Text and voice command interface for AMR mission control.",
    license="Proprietary",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "voice_text_cli = amr_voice.voice_text_cli:main",
            "voice_command_node = amr_voice.voice_text_cli:main",
        ],
    },
)
