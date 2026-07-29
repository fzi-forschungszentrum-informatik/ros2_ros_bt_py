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
import abc

from ros_bt_py.vendor.result import Result, Ok, Err

from ros_bt_py.data_types import GenericType, ReferenceType
from ros_bt_py.exceptions import BehaviorTreeException
from ros_bt_py.helpers import BTNodeState
from ros_bt_py.node import Leaf, define_bt_node
from ros_bt_py.node_config import NodeConfig


@define_bt_node(
    NodeConfig(
        inputs={
            "io_type": GenericType(),
            "in": ReferenceType("io_type"),
            "default": ReferenceType("io_type"),
        },
        outputs={"out": ReferenceType("io_type")},
        max_children=0,
    )
)
class IO(Leaf):
    """
    Base class for IO nodes in the tree.

    IO nodes have no children. Subclasses can define inputs and outputs,
    but never change `max_children`.
    """

    @abc.abstractmethod
    def _abstract_flag(self):
        # This is used to prevent the common base class from being picked up as a valid node
        pass

    def _do_setup(self) -> Result[BTNodeState, BehaviorTreeException]:
        return Ok(BTNodeState.IDLE)

    def _do_tick(self) -> Result[BTNodeState, BehaviorTreeException]:
        return (
            self.inputs.get_value("in")
            .or_else(lambda _: self.inputs.get_value("default"))
            .and_then(lambda val: self.outputs.set_value("out", val))
            .map(lambda _: BTNodeState.SUCCEEDED)
        )

    def _do_untick(self) -> Result[BTNodeState, BehaviorTreeException]:
        return Ok(BTNodeState.IDLE)

    def _do_shutdown(self) -> Result[BTNodeState, BehaviorTreeException]:
        return Ok(BTNodeState.SHUTDOWN)

    def _do_reset(self) -> Result[BTNodeState, BehaviorTreeException]:
        return Ok(BTNodeState.IDLE)


@define_bt_node(NodeConfig(inputs={}, outputs={}, max_children=0))
class IOInput(IO):
    """
    Explicitly marks the input of a subtree.

    If no input is connected to `in`, the value provided via the `default` input is used.
    """

    def _abstract_flag(self):
        pass


@define_bt_node(NodeConfig(inputs={}, outputs={}, max_children=0))
class IOOutput(IO):
    """
    Explicitly marks the output of a subtree.

    If no input is connected to `in`, the value provided via the `default` input is used.
    """

    def _abstract_flag(self):
        pass
