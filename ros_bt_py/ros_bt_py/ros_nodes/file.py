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
"""BT Node to open a file from disk and publish its contents."""
from rclpy.utilities import ament_index_python
import yaml

from ros_bt_py_interfaces.msg import Node as NodeMsg

from ros_bt_py.exceptions import BehaviorTreeException
from ros_bt_py.node import Leaf, define_bt_node
from ros_bt_py.node_config import NodeConfig


class LoadFileError(Exception):
    """Exception when the file cannot be loaded."""

    pass


def load_file(path):
    """Load a file from a ROS url."""
    file_path = ""
    if path.startswith("file://"):
        file_path = path[len("file://") :]
    elif path.startswith("package://"):
        package_name = path[len("package://") :].split("/", 1)[0]
        package_path = ament_index_python.get_package_share_directory(
            package_name=package_name
        )
        file_path = package_path + path[len("package://") + len(package_name) :]
    else:
        raise LoadFileError(
            f'File path "{path}" is malformed.'
            'It needs to start with either "file://" or "package://"'
        )
    try:
        data_file = open(file_path, "r")
    except IOError as ex:
        error_msg = f"Error opening file {file_path}: {str(ex)}"
        raise LoadFileError(error_msg)
    with data_file:
        try:
            data = yaml.safe_load(data_file)
        except yaml.YAMLError as ex:
            raise LoadFileError(f"Yaml error in file {file_path}: {str(ex)}")
    return data


@define_bt_node(
    NodeConfig(
        version="0.1.0",
        options={"file_path": str},
        inputs={},
        outputs={
            "load_success": bool,
            "load_error_msg": str,
            "content": list,
            "line_count": int,
        },
        max_children=0,
        optional_options=["something"],
    )
)
class YamlListOption(Leaf):
    """
    Load a yaml file from the location pointed to by `file_path`.

    This uses package:// and file:// style URIs.
    """

    def _do_setup(self):
        if not self.has_ros_node:
            error_msg = f"{self.name} has no reference to ROS node!"
            self.logerr(error_msg)
            raise BehaviorTreeException(error_msg)
        self.data = None
        self.outputs["load_success"] = False
        self.outputs["load_error_msg"] = ""

        try:
            data = load_file(self.options["file_path"])
            if data and isinstance(data, list):
                self.outputs["load_success"] = True
                self.data = data
            else:
                self.outputs["load_error_msg"] = "Yaml file should be a list"
        except LoadFileError as ex:
            self.outputs["load_error_msg"] = str(ex)

    def _do_tick(self):
        if self.data is not None:
            self.outputs["content"] = self.data
            self.outputs["line_count"] = len(self.data)

            return NodeMsg.SUCCEEDED
        return NodeMsg.FAILED

    def _do_untick(self):
        return NodeMsg.IDLE

    def _do_reset(self):
        return NodeMsg.IDLE

    def _do_shutdown(self):
        pass


@define_bt_node(
    NodeConfig(
        version="0.1.0",
        options={},
        inputs={"file_path": str},
        outputs={
            "load_success": bool,
            "load_error_msg": str,
            "content": list,
            "line_count": int,
        },
        max_children=0,
    )
)
class YamlListInput(Leaf):
    """
    Load a yaml file (list) from the location pointed to by `file_path`.

    This uses package:// and file:// style URIs.
    """

    def _do_setup(self):
        if not self.has_ros_node:
            error_msg = f"{self.name} has no reference to ROS node!"
            self.logerr(error_msg)
            raise BehaviorTreeException(error_msg)
        self.data = None

    def _do_tick(self):
        if self.inputs.is_updated("file_path"):
            self.outputs["load_success"] = False
            self.outputs["load_error_msg"] = ""
            try:
                data = load_file(self.inputs["file_path"])
            except LoadFileError as ex:
                self.outputs["load_error_msg"] = str(ex)
                return NodeMsg.FAILED

            if data and isinstance(data, list):
                self.data = data
                self.outputs["load_success"] = True
                self.outputs["content"] = self.data
                self.outputs["line_count"] = len(self.data)
            else:
                self.outputs["load_error_msg"] = "Yaml file should be a list"
        return NodeMsg.SUCCEEDED if self.outputs["load_success"] else NodeMsg.FAILED

    def _do_untick(self):
        return NodeMsg.IDLE

    def _do_reset(self):
        self.inputs.reset_updated()
        self.data = None
        return NodeMsg.IDLE

    def _do_shutdown(self):
        pass


@define_bt_node(
    NodeConfig(
        version="0.1.0",
        options={},
        inputs={"file_path": str},
        outputs={"load_success": bool, "load_error_msg": str, "content": dict},
        max_children=0,
    )
)
class YamlDictInput(Leaf):
    """
    Load a yaml file (dict) from the location pointed to by `file_path`.

    This uses package:// and file:// style URIs.
    """

    def _do_setup(self):
        if not self.has_ros_node:
            error_msg = f"{self.name} has no reference to ROS node!"
            self.logerr(error_msg)
            raise BehaviorTreeException(error_msg)
        self.data = None

    def _do_tick(self):
        if self.inputs.is_updated("file_path"):
            self.outputs["load_success"] = False
            self.outputs["load_error_msg"] = ""
            try:
                data = load_file(self.inputs["file_path"])
            except LoadFileError as ex:
                self.outputs["load_error_msg"] = str(ex)
                return NodeMsg.FAILED

            if data and isinstance(data, dict):
                self.data = data
                self.outputs["load_success"] = True
                self.outputs["content"] = self.data
            else:
                self.outputs["load_error_msg"] = "Yaml file should be a dict"
        return NodeMsg.SUCCEEDED if self.outputs["load_success"] else NodeMsg.FAILED

    def _do_untick(self):
        return NodeMsg.IDLE

    def _do_reset(self):
        self.inputs.reset_updated()
        self.data = None
        return NodeMsg.IDLE

    def _do_shutdown(self):
        pass
