from setuptools import setup

package_name = "AP4_init_test_package"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Fredrik Juthe",
    maintainer_email="Fredrik.Juthe@infotiv.se",
    description="Init test to verify all connections and ECUs of the AP4 gokart",
    license="None",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": ["talker = AP4_init_test_package.AP4_init_test_script:main"],
    },
)
