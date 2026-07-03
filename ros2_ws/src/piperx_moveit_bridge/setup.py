from setuptools import find_packages, setup


package_name = "piperx_moveit_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="TODO",
    maintainer_email="TODO@example.com",
    description="Bridge JSON pick requests into ROS2 MoveIt planning.",
    license="TODO",
    entry_points={
        "console_scripts": [
            "plan_from_json = piperx_moveit_bridge.plan_from_json:main",
        ],
    },
)
