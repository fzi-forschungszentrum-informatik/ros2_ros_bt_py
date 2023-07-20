# Copyright 2023 FZI Forschungszentrum Informatik

from setuptools import setup

package_name = "ros_bt_py"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="David Oberacker",
    maintainer_email="oberacker@fzi.de",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest", "pytest-cov"],
    entry_points={
        "console_scripts": ["tree_node = ros_bt_py.tree_node:main"],
    },
)
