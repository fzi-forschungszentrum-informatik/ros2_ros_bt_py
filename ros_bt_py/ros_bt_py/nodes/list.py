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

from ros_bt_py.vendor.result import Result, Ok, Err, do

from ros_bt_py.data_types import (
    GenericType,
    IntType,
    ReferenceListType,
    ReferenceType,
)
from ros_bt_py.exceptions import BehaviorTreeException
from ros_bt_py.helpers import BTNodeState
from ros_bt_py.node import Leaf, Decorator, define_bt_node
from ros_bt_py.node_config import NodeConfig


@define_bt_node(
    NodeConfig(
        inputs={
            "element_type": GenericType(),
            "list": ReferenceListType(reference="element_type"),
        },
        outputs={"length": IntType()},
        max_children=0,
    )
)
class ListLength(Leaf):
    """Compute list length."""

    def _do_setup(self) -> Result[BTNodeState, BehaviorTreeException]:
        return Ok(BTNodeState.IDLE)

    def _do_tick(self) -> Result[BTNodeState, BehaviorTreeException]:
        return (
            self.inputs.get_value_as("list", list)
            .and_then(lambda li: self.outputs.set_value("length", len(li)))
            .map(lambda _: BTNodeState.SUCCEEDED)
        )

    def _do_shutdown(self) -> Result[BTNodeState, BehaviorTreeException]:
        return Ok(BTNodeState.SHUTDOWN)

    def _do_reset(self) -> Result[BTNodeState, BehaviorTreeException]:
        return Ok(BTNodeState.IDLE)

    def _do_untick(self) -> Result[BTNodeState, BehaviorTreeException]:
        return Ok(BTNodeState.IDLE)


@define_bt_node(
    NodeConfig(
        inputs={
            "element_type": GenericType(),
            "index": IntType(min_value=0),
            "list": ReferenceListType(reference="element_type"),
            "element": ReferenceType(reference="element_type"),
        },
        outputs={"list": ReferenceListType(reference="element_type")},
        max_children=0,
    )
)
class InsertInList(Leaf):
    """Return a new list with the inserted element."""

    def _do_setup(self) -> Result[BTNodeState, BehaviorTreeException]:
        return Ok(BTNodeState.IDLE)

    def _do_tick(self) -> Result[BTNodeState, BehaviorTreeException]:
        def insert_and_return(li: list, index: int, val):
            li.insert(index, val)
            return li

        return (
            do(
                Ok(insert_and_return(li, i, e))
                for li in self.inputs.get_value_as("list", list)
                for i in self.inputs.get_value_as("index", int)
                for e in self.inputs.get_value("element")
            )
            .and_then(lambda val: self.outputs.set_value("list", val))
            .map(lambda _: BTNodeState.SUCCEEDED)
        )

    def _do_shutdown(self) -> Result[BTNodeState, BehaviorTreeException]:
        return Ok(BTNodeState.SHUTDOWN)

    def _do_reset(self) -> Result[BTNodeState, BehaviorTreeException]:
        return Ok(BTNodeState.IDLE)

    def _do_untick(self) -> Result[BTNodeState, BehaviorTreeException]:
        return Ok(BTNodeState.IDLE)


@define_bt_node(
    NodeConfig(
        inputs={
            "compare_type": GenericType(),
            "list": ReferenceListType(reference="compare_type"),
            "in": ReferenceType(reference="compare_type"),
        },
        outputs={},
        max_children=0,
    )
)
class IsInList(Leaf):
    """
    Check if `in` is in provided list.

    Will succeed if `in` is in `list` and fail otherwise
    """

    def _do_setup(self) -> Result[BTNodeState, BehaviorTreeException]:
        return Ok(BTNodeState.IDLE)

    def _do_tick(self) -> Result[BTNodeState, BehaviorTreeException]:
        return do(
            Ok(e in li)
            for e in self.inputs.get_value("in")
            for li in self.inputs.get_value_as("list", list)
        ).map(lambda res: BTNodeState.SUCCEEDED if res else BTNodeState.FAILED)

    def _do_untick(self) -> Result[BTNodeState, BehaviorTreeException]:
        return Ok(BTNodeState.IDLE)

    def _do_reset(self) -> Result[BTNodeState, BehaviorTreeException]:
        return Ok(BTNodeState.IDLE)

    def _do_shutdown(self) -> Result[BTNodeState, BehaviorTreeException]:
        return Ok(BTNodeState.SHUTDOWN)


@define_bt_node(
    NodeConfig(
        inputs={
            "item_type": GenericType(),
            "list": ReferenceListType(reference="item_type"),
        },
        outputs={"list_item": ReferenceType(reference="item_type")},
        max_children=1,
    )
)
class IterateList(Decorator):
    """
    Iterate through list provided as input.

    The elements in the list are iterated on the output list_item.
    The iteration goes when the decorated child returns a success.
    This node returns running until the iteration is done.
    If it managed to iterate through the list, it returns success.
    If the decorated child returned failure, it fails.
    """

    def _do_setup(self) -> Result[BTNodeState, BehaviorTreeException]:
        self.reset_counter()
        for child in self.children:
            return child.setup()
        return Ok(BTNodeState.IDLE)

    def reset_counter(self):
        self.output_changed = True
        self.counter = 0

    def _do_tick(self) -> Result[BTNodeState, BehaviorTreeException]:
        match self.inputs.any_updated("list"):
            case Err(e):
                return Err(e)
            case Ok(b):
                updated = b
        if updated:
            self.loginfo("Input list changed - resetting iterator")
            self.reset_counter()

        match self.inputs.get_value_as("list", list):
            case Err(e):
                return Err(e)
            case Ok(l):
                in_list = l

        # if no items in 'list' directly succeed
        if len(in_list) > 0:
            match self.outputs.set_value("list_item", in_list[self.counter]):
                case Err(e):
                    return Err(e)
                case Ok(None):
                    pass
        else:
            self.loginfo("Nothing to iterate, input list is empty")
            return Ok(BTNodeState.SUCCEEDED)

        if len(self.children) == 0:
            self.counter += 1
            if self.counter == len(in_list):
                self.reset_counter()
                return Ok(BTNodeState.SUCCEEDED)
        else:
            if self.output_changed:
                # let one tick go for the tree to digest our new output before childs are ticked
                self.output_changed = False
                return Ok(BTNodeState.RUNNING)
            for child in self.children:
                match child.tick():
                    case Err(e):
                        return Err(e)
                    case Ok(s):
                        child_state = s
                if child_state == BTNodeState.SUCCEEDED:
                    # we only increment the counter when the child succeeded
                    self.counter += 1
                    self.output_changed = True
                    if self.counter == len(in_list):
                        self.reset_counter()
                        return Ok(BTNodeState.SUCCEEDED)
                elif child_state == BTNodeState.FAILED:
                    # child failed: we failed
                    return Ok(BTNodeState.FAILED)
        return Ok(BTNodeState.RUNNING)

    def _do_untick(self) -> Result[BTNodeState, BehaviorTreeException]:
        for child in self.children:
            return child.untick()
        return Ok(BTNodeState.IDLE)

    def _do_reset(self) -> Result[BTNodeState, BehaviorTreeException]:
        self.reset_counter()
        for child in self.children:
            return child.reset()
        return Ok(BTNodeState.IDLE)

    def _do_shutdown(self) -> Result[BTNodeState, BehaviorTreeException]:
        return Ok(BTNodeState.SHUTDOWN)
