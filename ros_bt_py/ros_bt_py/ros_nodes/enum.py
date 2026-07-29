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

from ros_bt_py.data_types import (
    DataContainer,
    RosComponentType,
    GENERIC_TYPE_MAP,
    get_iotype_for_dict,
)
from ros_bt_py.exceptions import BehaviorTreeException, NodeConfigError

from ros_bt_py.node import Leaf, define_bt_node
from ros_bt_py.node_config import NodeConfig

from ros_bt_py.helpers import BTNodeState
from ros_bt_py.ros_helpers import get_message_constant_fields


@define_bt_node(
    NodeConfig(
        inputs={"ros_message_type": RosComponentType()},
        outputs={},
        max_children=0,
    )
)
class EnumFields(Leaf):
    """
    Expose the constants in a ROS message as multiple output fields.

    The outputs will be named after the fields in the ROS message
    """

    def add_extra_outputs(self) -> Result[dict[str, DataContainer], NodeConfigError]:
        match self.inputs.get_value("ros_message_type"):
            case Err(e):
                return Err(e)
            case Ok(v):
                self._message_class = v

        match get_message_constant_fields(self._message_class):
            case Err(e):
                return Err(e)
            case Ok(c):
                self._constants = c

        node_outputs = {}
        for field in self._constants:
            field_type = type(getattr(self._message_class, field))
            if field not in GENERIC_TYPE_MAP.keys():
                return Err(NodeConfigError(f"Field {field} has an incompatible type"))
            match get_iotype_for_dict(GENERIC_TYPE_MAP[field_type]):
                case Err(e):
                    return Err(NodeConfigError(e))
                case Ok(c):
                    node_outputs[field] = c
        return Ok(node_outputs)

    def _do_setup(self) -> Result[BTNodeState, BehaviorTreeException]:
        self.first_tick = True
        return Ok(BTNodeState.IDLE)

    def _do_tick(self) -> Result[BTNodeState, BehaviorTreeException]:
        return self.outputs.set_multiple_values(
            **{field: getattr(self._message_class, field) for field in self._constants}
        ).map(lambda _: BTNodeState.SUCCEEDED)

    def _do_untick(self) -> Result[BTNodeState, BehaviorTreeException]:
        return Ok(BTNodeState.IDLE)

    def _do_shutdown(self) -> Result[BTNodeState, BehaviorTreeException]:
        return Ok(BTNodeState.SHUTDOWN)

    def _do_reset(self) -> Result[BTNodeState, BehaviorTreeException]:
        self.first_tick = True
        return Ok(BTNodeState.IDLE)
