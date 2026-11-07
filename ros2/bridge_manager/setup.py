from setuptools import setup

package_name = "bridge_manager"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml", "LICENSE"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    tests_require=["pytest"],
    maintainer="Abhimanyu Jadli",
    url="https://github.com/rosnect/ros2embedded",
    maintainer_email="rosnect.dev@tutamail.com",
    description="Manager node for launching and tracking ROS2 embedded bridge nodes.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "bridge_manager = bridge_manager.manager_node:main",
        ],
    },
)
