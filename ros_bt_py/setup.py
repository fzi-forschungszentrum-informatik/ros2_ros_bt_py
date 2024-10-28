# Copyright 2023 FZI Forschungszentrum Informatik
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the FZI Forschungszentrum Informatik nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
import os
import xml.etree.ElementTree as ET
from glob import glob
from setuptools import setup
from generate_parameter_library_py.setup_helper import generate_parameter_module

generate_parameter_module(
    "parameters",  # python module name for parameter library
    "config/tree_node_parameters.yaml",  # path to input yaml file
)

package_xml_path = os.path.join(os.getcwd(), "package.xml")
package_xmldata = ET.parse(package_xml_path)

package_name = package_xmldata.find("name").text
version_str = package_xmldata.find("version").text
description_str = package_xmldata.find("description").text
license_str = package_xmldata.find("license").text
maintainer = package_xmldata.find("maintainer")
maintainer_name_str = maintainer.text
maintainer_email_str = maintainer.attrib["email"]

setup(
    name=package_name,
    version=version_str,
    packages=[
        "ros_bt_py",
        "ros_bt_py.nodes",
        "ros_bt_py.ros_nodes",
    ],
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
    zip_safe=True,
    maintainer=maintainer_name_str,
    maintainer_email=maintainer_email_str,
    description=description_str,
    license=license_str,
    tests_require=["pytest", "pytest-cov"],
    entry_points={
        "console_scripts": ["tree_node = ros_bt_py.tree_node:main"],
    },
)
