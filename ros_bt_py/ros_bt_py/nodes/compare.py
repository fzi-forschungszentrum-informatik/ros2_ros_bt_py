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

from ros_bt_py.vendor.result import Err, Result, Ok, do

from ros_bt_py.data_types import (
    GenericType,
    ReferenceType,
)
from ros_bt_py.exceptions import BehaviorTreeException
from ros_bt_py.helpers import BTNodeState
from ros_bt_py.node import Leaf, define_bt_node
from ros_bt_py.node_config import NodeConfig


@define_bt_node(
    NodeConfig(
        inputs={
            "compare_type": GenericType(),
            "a": ReferenceType(reference="compare_type", is_static=False),
            "b": ReferenceType(reference="compare_type"),
        },
        outputs={},
        max_children=0,
    )
)
class Compare(Leaf):
    """
    Compares `a` and `b`.

    Will succeed if `a == b` and fail otherwise
    """

    def _do_setup(self) -> Result[BTNodeState, BehaviorTreeException]:
        return Ok(BTNodeState.IDLE)

    def _do_tick(self) -> Result[BTNodeState, BehaviorTreeException]:
        return do(
            Ok(a == b)
            for a in self.inputs.get_value("a")
            for b in self.inputs.get_value("b")
        ).map(lambda res: BTNodeState.SUCCEEDED if res else BTNodeState.FAILED)

    def _do_untick(self) -> Result[BTNodeState, BehaviorTreeException]:
        # Nothing to do
        return Ok(BTNodeState.IDLE)

    def _do_reset(self) -> Result[BTNodeState, BehaviorTreeException]:
        return Ok(BTNodeState.IDLE)

    def _do_shutdown(self) -> Result[BTNodeState, BehaviorTreeException]:
        # Nothing to do
        return Ok(BTNodeState.SHUTDOWN)


@define_bt_node(
    NodeConfig(
        inputs={
            "compare_type": GenericType(),
            "a": ReferenceType(reference="compare_type", is_static=False),
            "b": ReferenceType(reference="compare_type"),
        },
        outputs={},
        max_children=0,
    )
)
class CompareNewOnly(Leaf):
    """
    Compares `a` and `b`, but only if at least one of them has been updated this tick.

    Will succeed if `a == b` and fail otherwise
    """

    def _do_setup(self) -> Result[BTNodeState, BehaviorTreeException]:
        return Ok(BTNodeState.IDLE)

    def _do_tick(self) -> Result[BTNodeState, BehaviorTreeException]:
        match self.inputs.any_updated("a", "b"):
            case Err(e):
                return Err(e)
            case Ok(b):
                updated = b
        if not updated:
            return Ok(BTNodeState.RUNNING)

        return do(
            Ok(a == b)
            for a in self.inputs.get_value("a")
            for b in self.inputs.get_value("b")
        ).map(lambda res: BTNodeState.SUCCEEDED if res else BTNodeState.FAILED)

    def _do_untick(self) -> Result[BTNodeState, BehaviorTreeException]:
        # Nothing to do
        return Ok(BTNodeState.IDLE)

    def _do_reset(self) -> Result[BTNodeState, BehaviorTreeException]:
        # Nothing to do
        return Ok(BTNodeState.IDLE)

    def _do_shutdown(self) -> Result[BTNodeState, BehaviorTreeException]:
        # Nothing to do
        return Ok(BTNodeState.SHUTDOWN)


@define_bt_node(
    NodeConfig(
        inputs={
            "operand_type": GenericType(valid_types=[int, float]),
            "a": ReferenceType(reference="operand_type", is_static=False),
            "b": ReferenceType(reference="operand_type"),
        },
        outputs={},
        max_children=0,
    )
)
class ALessThanB(Leaf):
    """
    Compares `a` and `b`.

    Will succeed if `a < b` and fail otherwise
    """

    def _do_setup(self) -> Result[BTNodeState, BehaviorTreeException]:
        return Ok(BTNodeState.IDLE)

    def _do_tick(self) -> Result[BTNodeState, BehaviorTreeException]:
        return do(
            Ok(a < b)
            for a in self.inputs.get_value("a")
            for b in self.inputs.get_value("b")
        ).map(lambda res: BTNodeState.SUCCEEDED if res else BTNodeState.FAILED)

    def _do_untick(self) -> Result[BTNodeState, BehaviorTreeException]:
        # Nothing to do
        return Ok(BTNodeState.IDLE)

    def _do_reset(self) -> Result[BTNodeState, BehaviorTreeException]:
        # Nothing to do
        return Ok(BTNodeState.IDLE)

    def _do_shutdown(self) -> Result[BTNodeState, BehaviorTreeException]:
        # Nothing to do
        return Ok(BTNodeState.SHUTDOWN)
