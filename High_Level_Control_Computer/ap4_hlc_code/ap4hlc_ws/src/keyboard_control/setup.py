from setuptools import find_packages, setup

package_name = "keyboard_control"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="anton",
    maintainer_email="anton.hill@infotiv.se",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "keyboard_control_node= keyboard_control.keyboard_control:main",
            "lidar_output_node = keyboard_control.lidar_output:main",
        ],
    },
)
