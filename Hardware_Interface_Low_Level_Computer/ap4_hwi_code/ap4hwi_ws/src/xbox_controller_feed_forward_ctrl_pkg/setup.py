from setuptools import setup
from glob import glob
import os

package_name = "xbox_controller_feed_forward_ctrl_pkg"
setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="root",
    maintainer_email="erik.magnusson@infotiv.se",
    description="Package to control gokart feedforward using xbox controller",
    license="tokill",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "feed_forward_node = xbox_controller_feed_forward_ctrl_pkg.feed_forward_controller_script:main",
        ],
    },
)
