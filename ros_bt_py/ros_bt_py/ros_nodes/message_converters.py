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

from ros_bt_py.vendor.result import Result, Ok, Err

from ros_bt_py.data_types import (
    DataContainer,
    RosTopicType,
    ReferenceType,
    get_message_field_io_type,
)
from ros_bt_py.node import Leaf, define_bt_node
from ros_bt_py.node_config import NodeConfig
from ros_bt_py.exceptions import BehaviorTreeException, NodeConfigError
from ros_bt_py.helpers import BTNodeState


@define_bt_node(
    NodeConfig(
        inputs={
            "input_type": RosTopicType(),
            "in": ReferenceType(reference="input_type"),
        },
        outputs={},
        max_children=0,
    )
)
class MessageToFields(Leaf):
    def add_extra_outputs(self) -> Result[dict[str, DataContainer], NodeConfigError]:
        match self.inputs.get_value_as("input_type", type):
            case Err(e):
                return Err(e)
            case Ok(t):
                self.msg_type = t
        outputs = {}
        for (
            field_name,
            field_type,
        ) in self.msg_type.get_fields_and_field_types().items():
            match get_message_field_io_type(field_type):
                case Err(e):
                    return Err(NodeConfigError(e))
                case Ok(t):
                    outputs[field_name] = t
        return Ok(outputs)

    def _do_setup(self) -> Result[BTNodeState, BehaviorTreeException]:
        self.msg_fields = list(self.msg_type.get_fields_and_field_types().keys())
        return Ok(BTNodeState.IDLE)

    def _do_tick(self) -> Result[BTNodeState, BehaviorTreeException]:
        match self.inputs.get_value_as("in", self.msg_type):
            case Err(e):
                return Err(e)
            case Ok(m):
                msg = m
        return self.outputs.set_multiple_values(
            **{field: getattr(msg, field) for field in self.msg_fields}
        ).map(lambda _: BTNodeState.SUCCEEDED)

    def _do_untick(self) -> Result[BTNodeState, BehaviorTreeException]:
        return Ok(BTNodeState.IDLE)

    def _do_shutdown(self) -> Result[BTNodeState, BehaviorTreeException]:
        return Ok(BTNodeState.SHUTDOWN)

    def _do_reset(self) -> Result[BTNodeState, BehaviorTreeException]:
        return Ok(BTNodeState.IDLE)


@define_bt_node(
    NodeConfig(
        inputs={"output_type": RosTopicType()},
        outputs={"out": ReferenceType(reference="output_type")},
        max_children=0,
    )
)
class FieldsToMessage(Leaf):
    """
    Take multiple fields as input and outputs a ROS message.

    The inputs will be named after the fields in the output message
    """

    def add_extra_inputs(self) -> Result[dict[str, DataContainer], NodeConfigError]:
        match self.inputs.get_value_as("output_type", type):
            case Err(e):
                return Err(e)
            case Ok(t):
                self.msg_type = t
        inputs = {}
        for (
            field_name,
            field_type,
        ) in self.msg_type.get_fields_and_field_types().items():
            match get_message_field_io_type(field_type):
                case Err(e):
                    return Err(NodeConfigError(e))
                case Ok(t):
                    inputs[field_name] = t
        return Ok(inputs)

    def _do_setup(self) -> Result[BTNodeState, BehaviorTreeException]:
        self.msg_fields = list(self.msg_type.get_fields_and_field_types().keys())
        return Ok(BTNodeState.IDLE)

    def _do_tick(self) -> Result[BTNodeState, BehaviorTreeException]:
        msg = self.msg_type()
        for field in self.msg_fields:
            match self.inputs.get_value(field):
                case Err(e):
                    return Err(e)
                case Ok(v):
                    value = v
            setattr(msg, field, value)

        return self.outputs.set_value("out", msg).map(lambda _: BTNodeState.SUCCEEDED)

    def _do_untick(self) -> Result[BTNodeState, BehaviorTreeException]:
        return Ok(BTNodeState.IDLE)

    def _do_shutdown(self) -> Result[BTNodeState, BehaviorTreeException]:
        return Ok(BTNodeState.SHUTDOWN)

    def _do_reset(self) -> Result[BTNodeState, BehaviorTreeException]:
        return Ok(BTNodeState.IDLE)
