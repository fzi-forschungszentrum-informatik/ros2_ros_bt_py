# Copyright 2025 FZI Forschungszentrum Informatik
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

from ros_bt_py.vendor.result import Result, Ok, Err, do

from ros_bt_py.data_types import (
    DataContainer,
    RosComponentType,
    GENERIC_TYPE_MAP,
    get_iotype_for_dict,
)
from ros_bt_py.exceptions import BehaviorTreeException, NodeConfigError

from ros_bt_py.node import FlowControl, define_bt_node
from ros_bt_py.node_config import NodeConfig

from ros_bt_py.helpers import BTNodeState
from ros_bt_py.ros_helpers import get_message_constant_fields

from ros_bt_py_interfaces.msg import NodeState


@define_bt_node(
    NodeConfig(
        inputs={"ros_message_type": RosComponentType(value=NodeState)},
        outputs={},
        max_children=None,
    )
)
class EnumSwitch(FlowControl):

    def add_extra_inputs(self) -> Result[dict[str, DataContainer], NodeConfigError]:
        match self.inputs.get_value("ros_message_type"):
            case Err(e):
                return Err(e)
            case Ok(v):
                self._message_class = v

        match get_message_constant_fields(self._message_class):
            case Err(e):
                return Err(e)
            case Ok(li):
                self.possible_children = li
        if not self.possible_children:
            return Err(NodeConfigError(f"{self._message_class} has no constant fields"))

        # Check if all constants have equal type and raise error if not. Otherwise set input to
        # constant type
        pchild_types = [
            type(getattr(self._message_class, field))
            for field in self.possible_children
        ]
        if len(set(pchild_types)) > 1:
            return Err(
                NodeConfigError(
                    f"{self._message_class} contains constant fields of multiple types"
                )
            )

        if pchild_types[0] not in GENERIC_TYPE_MAP.keys():
            return Err(
                NodeConfigError(f"Type {pchild_types[0]} is not a valid IO type")
            )
        match get_iotype_for_dict(GENERIC_TYPE_MAP[pchild_types[0]]):
            case Err(e):
                return Err(NodeConfigError(e))
            case Ok(c):
                case_type = c
        case_type.is_static = False

        return Ok({"case": case_type})

    def _do_setup(self) -> Result[BTNodeState, BehaviorTreeException]:
        # Create possible child dict
        self.msg = self._message_class()
        self.pchild_dict = {
            getattr(self.msg, entry): entry for entry in self.possible_children
        }

        self.child_map = {child.name.split(".")[-1]: child for child in self.children}
        for child in self.children:
            if child.name.split(".")[-1] not in self.possible_children:
                self.logerr("Unwanted child detected please fix name")
            match child.setup():
                case Err(e):
                    return Err(e)
                case Ok(_):
                    pass
        return Ok(BTNodeState.IDLE)

    def _do_tick(self) -> Result[BTNodeState, BehaviorTreeException]:
        match self.inputs.get_value("case"):
            case Err(e):
                return Err(e)
            case Ok(c):
                case = c

        if case in self.pchild_dict.keys():
            e_name = self.pchild_dict[case]
        else:
            self.logwarn("Input did not match possible children.")
            return Ok(BTNodeState.FAILED)

        if e_name not in self.child_map:
            self.logwarn(
                "Ticking without matching child. Is this really what you want?"
            )
            return Ok(BTNodeState.FAILED)

        # If we've previously succeeded or failed, reset all children
        if self.state in [BTNodeState.SUCCEEDED, BTNodeState.FAILED]:
            for child in self.children:
                match child.reset():
                    case Err(e):
                        return Err(e)
                    case Ok(_):
                        pass

        for child_name, child in self.child_map.items():
            if child_name == e_name:
                continue
            match child.untick():
                case Err(e):
                    return Err(e)
                case Ok(_):
                    pass
        return self.child_map[e_name].tick()

    def _do_untick(self) -> Result[BTNodeState, BehaviorTreeException]:
        for child in self.children:
            match child.untick():
                case Err(e):
                    return Err(e)
                case Ok(_):
                    pass
        return Ok(BTNodeState.IDLE)

    def _do_reset(self) -> Result[BTNodeState, BehaviorTreeException]:
        for child in self.children:
            match child.reset():
                case Err(e):
                    return Err(e)
                case Ok(_):
                    pass
        return Ok(BTNodeState.IDLE)

    def _do_shutdown(self) -> Result[BTNodeState, BehaviorTreeException]:
        return Ok(BTNodeState.SHUTDOWN)
