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
from typing import Optional

from ros_bt_py.vendor.result import Result, Ok, Err

from ros_bt_py_interfaces.msg import UtilityBounds
from ros_bt_py.exceptions import BehaviorTreeException

from ros_bt_py.data_types import (
    GenericType,
    StringType,
    ReferenceType,
)
from ros_bt_py.helpers import BTNodeState
from ros_bt_py.node import Leaf, define_bt_node
from ros_bt_py.node_config import NodeConfig


@define_bt_node(
    NodeConfig(
        inputs={
            "param_name": StringType(),
            "param_type": GenericType(valid_types=[bool, int, float, bytes, list]),
            "default_value": ReferenceType(reference="param_type"),
        },
        outputs={"param": ReferenceType(reference="param_type")},
        max_children=0,
    )
)
class RosParam(Leaf):
    """Read a parameter from the ROS parameter server."""

    def _do_setup(self) -> Result[BTNodeState, BehaviorTreeException]:
        if not self.has_ros_node:
            error_msg = (
                f"RosParamOption node {self.name} does not have ROS node reference!"
            )
            self.logerr(error_msg)
            return Err(BehaviorTreeException(error_msg))
        return Ok(BTNodeState.IDLE)

    def _do_tick(self) -> Result[BTNodeState, BehaviorTreeException]:
        match self.inputs.get_value_as("param_name", str):
            case Err(e):
                return Err(e)
            case Ok(s):
                param_name = s
        if not self.ros_node.has_parameter(param_name):
            return Ok(BTNodeState.FAILED)
        else:
            param = self.ros_node.get_parameter(param_name)
            param_value = param.value
            if param_value is None:
                return (
                    self.inputs.get_value("default_value")
                    .and_then(lambda val: self.outputs.set_value("param", val))
                    .map(lambda _: BTNodeState.SUCCEEDED)
                )
            else:
                return self.outputs.set_value("param", param_value).map(
                    lambda _: BTNodeState.SUCCEEDED
                )

    def _do_shutdown(self) -> Result[BTNodeState, BehaviorTreeException]:
        return Ok(BTNodeState.SHUTDOWN)

    def _do_reset(self) -> Result[BTNodeState, BehaviorTreeException]:
        return Ok(BTNodeState.IDLE)

    def _do_untick(self) -> Result[BTNodeState, BehaviorTreeException]:
        return Ok(BTNodeState.IDLE)

    def _do_calculate_utility(self) -> Result[UtilityBounds, BehaviorTreeException]:
        return Ok(UtilityBounds())
