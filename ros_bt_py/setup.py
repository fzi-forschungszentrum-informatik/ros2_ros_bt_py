# Copyright 2023 FZI Forschungszentrum Informatik

import os
from glob import glob
from setuptools import setup
from generate_parameter_library_py.setup_helper import generate_parameter_module

package_name = "ros_bt_py"

generate_parameter_module(
    "parameters",  # python module name for parameter library
    "config/tree_node_parameters.yaml",  # path to input yaml file
)

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "config"),
            glob(os.path.join("config", "*")),
        ),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*launch.[pxy][yma]*")),
        ),
    ],
    install_requires=["setuptools"],
    package_dir={"": "src/"},
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
