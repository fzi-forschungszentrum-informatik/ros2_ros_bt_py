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
from ros_bt_py_interfaces.msg import Node as NodeMsg
from ros_bt_py_interfaces.msg import UtilityBounds
from ros_bt_py.exceptions import BehaviorTreeException

from ros_bt_py.node import Leaf, define_bt_node
from ros_bt_py.node_config import NodeConfig, OptionRef
from ros_bt_py.custom_types import TypeWrapper, TYPE_BUILTIN


@define_bt_node(
    NodeConfig(
        version="0.1.0",
        options={
            "param_type": TypeWrapper(type, info=TYPE_BUILTIN),
            "default_value": OptionRef("param_type"),
            "param_name": str,
        },
        inputs={},
        outputs={"param": OptionRef("param_type")},
        max_children=0,
    )
)
class RosParamOption(Leaf):
    """Read a parameter from the ROS parameter server."""

    def _do_setup(self):
        if not self.has_ros_node:
            error_msg = (
                f"RosParamOption node {self.name} does not have ROS node reference!"
            )
            self.logerr(error_msg)
            raise BehaviorTreeException(error_msg)

    def _do_tick(self):
        if not self.ros_node.has_parameter(self.options["param_name"]):
            return NodeMsg.FAILURE
        else:
            param = self.ros_node.get_parameter(self.options["param_name"])
            param_value = param.value
            if param_value is None:
                self.outputs["param"] = self.options["default_value"]
                return NodeMsg.SUCCEEDED
            else:
                if isinstance(param_value, self.options["param_type"]):
                    self.outputs["param"] = param_value
                    return NodeMsg.SUCCEEDED
                else:
                    return NodeMsg.FAILED

    def _do_shutdown(self):
        pass

    def _do_reset(self):
        return NodeMsg.IDLE

    def _do_untick(self):
        return NodeMsg.IDLE

    def _do_calculate_utility(self):
        return UtilityBounds()


@define_bt_node(
    NodeConfig(
        version="0.1.0",
        options={
            "param_type": TypeWrapper(type, info=TYPE_BUILTIN),
            "param_name": str,
        },
        inputs={
            "default_value": OptionRef("param_type"),
        },
        outputs={"param": OptionRef("param_type")},
        max_children=0,
    )
)
class RosParamOptionDefaultInput(Leaf):
    """Read a parameter from the ROS parameter server."""

    def _do_setup(self):
        if not self.has_ros_node:
            error_msg = (
                f"RosParamOptionDefaultInput node {self.name} "
                "does not have ROS node reference!"
            )
            self.logerr(error_msg)
            raise BehaviorTreeException(error_msg)

    def _do_tick(self):
        if not self.ros_node.has_parameter(self.options["param_name"]):
            self.outputs["param"] = self.inputs["default_value"]
            return NodeMsg.FAILURE
        else:
            param = self.ros_node.get_parameter(self.options["param_name"])
            param_value = param.value
            if param_value is None:
                self.outputs["param"] = self.inputs["default_value"]
                return NodeMsg.SUCCEEDED
            else:
                if isinstance(param_value, self.options["param_type"]):
                    self.outputs["param"] = param_value
                    return NodeMsg.SUCCEEDED
                else:
                    return NodeMsg.FAILED

    def _do_shutdown(self):
        pass

    def _do_reset(self):
        return NodeMsg.IDLE

    def _do_untick(self):
        return NodeMsg.IDLE

    def _do_calculate_utility(self):
        return UtilityBounds()


@define_bt_node(
    NodeConfig(
        version="0.1.0",
        options={
            "param_type": TypeWrapper(type, info=TYPE_BUILTIN), 
            "default_value": OptionRef("param_type")
        },
        inputs={"param_name": str},
        outputs={"param": OptionRef("param_type")},
        max_children=0,
    )
)
class RosParamInput(Leaf):
    """Read a parameter from the ROS parameter server."""

    def _do_setup(self):
        if not self.has_ros_node:
            error_msg = (
                f"RosParamInput node {self.name} does not have ROS node reference!"
            )
            self.logerr(error_msg)
            raise BehaviorTreeException(error_msg)

    def _do_tick(self):
        if not self.ros_node.has_parameter(self.inputs["param_name"]):
            self.outputs["param"] = self.options["default_value"]
            return NodeMsg.FAILURE
        else:
            param = self.ros_node.get_parameter(self.inputs["param_name"])
            param_value = param.value
            if param_value is None:
                self.outputs["param"] = self.options["default_value"]
                return NodeMsg.SUCCEEDED
            else:
                if isinstance(param_value, self.options["param_type"]):
                    self.outputs["param"] = param_value
                    return NodeMsg.SUCCEEDED
                else:
                    return NodeMsg.FAILED

    def _do_shutdown(self):
        pass

    def _do_reset(self):
        return NodeMsg.IDLE

    def _do_untick(self):
        return NodeMsg.IDLE

    def _do_calculate_utility(self):
        return UtilityBounds()
