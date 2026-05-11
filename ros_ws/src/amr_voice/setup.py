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
    description="MCP-oriented voice intent helpers for AMR operator input.",
    license="Proprietary",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "wake_word_node = amr_voice.wake_word_node:main",
            "vad_node = amr_voice.vad_node:main",
        ],
    },
)
