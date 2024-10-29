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

from ros_bt_py.node import FlowControl, define_bt_node
from ros_bt_py.node_config import NodeConfig


@define_bt_node(
    NodeConfig(version="0.1.0", options={}, inputs={}, outputs={}, max_children=None)
)
class Sequence(FlowControl):
    """
    Flow control node that succeeds when all children succeed.

    At every call of :meth:`tick`, it ticks all of its children in
    order until one of three things happens:

    1. A node returns FAILED:

       In this case, the Sequence also returns FAILED, and calls
       :meth:`ros_bt_py.node.Node.untick` on all remaining children.

    2. A node returns RUNNING:

       Just as with FAILED, the Sequence will also return RUNNING, but
       not call :meth:`ros_bt_py.node.Node.untick` on the remaining
       children until the RUNNING node produces a result. This
       prevents thrashing when there's multiple nodes that take a tick
       or two to produce a result.

    3. All nodes return SUCCEEDED:

       The Sequence will also return SUCCEEDED.

    *Special case:*

    If a Sequence has no children, its :meth:`tick` method will always
    return SUCCEEDED.

    """

    def _do_setup(self):
        for child in self.children:
            child.setup()

    def _do_tick(self):
        if not self.children:
            self.logwarn("Ticking without children. Is this really what you want?")
            return NodeMsg.SUCCEEDED

        # If we've previously succeeded or failed, untick all children
        if self.state in [NodeMsg.SUCCEEDED, NodeMsg.FAILED]:
            for child in self.children:
                child.reset()

        # Tick children until one returns FAILED or RUNNING
        result = NodeMsg.FAILED
        for index, child in enumerate(self.children):
            result = child.tick()
            if result != NodeMsg.SUCCEEDED:
                # For all states other than RUNNING...
                if result != NodeMsg.RUNNING:
                    # ...untick all children after the one that hasn't
                    # succeeded
                    for untick_child in self.children[index + 1 :]:
                        untick_child.untick()
                break

        return result

    def _do_untick(self):
        for child in self.children:
            child.untick()
        return NodeMsg.IDLE

    def _do_reset(self):
        for child in self.children:
            child.reset()
        return NodeMsg.IDLE

    def _do_shutdown(self):
        for child in self.children:
            child.shutdown()

    def _do_calculate_utility(self):
        return calculate_utility_sequence(self.children)


@define_bt_node(
    NodeConfig(version="0.1.0", options={}, inputs={}, outputs={}, max_children=None)
)
class MemorySequence(FlowControl):
    """
    Flow control node that succeeds when all children succeed and has a memory.

    At every call of :meth:`tick`, it ticks all of its children in
    order until one of three things happens:

    1. A node returns FAILED:

       In this case, the Sequence also returns FAILED, and calls
       :meth:`ros_bt_py.node.Node.untick` on all remaining children.

    2. A node returns RUNNING:

       Just as with FAILED, the Sequence will also return RUNNING, but
       not call :meth:`ros_bt_py.node.Node.untick` on the remaining
       children until the RUNNING node produces a result. This
       prevents thrashing when there's multiple nodes that take a tick
       or two to produce a result.

    3. All nodes return SUCCEEDED:

       The Sequence will also return SUCCEEDED.


    The *Memory* part of the node means that after a child returns
    RUNNING, the execution will start at that same child on the next
    tick. This means that changes in the previous nodes' outcomes will
    not influence the execution of later children until either a) this
    node receives an `untick()` or b) the sequence returns SUCCEEDED
    or FAILED as described above.

    *Special case:*

    If a Sequence has no children, its :meth:`tick` method will always
    return SUCCEEDED.

    """

    def _do_setup(self):
        self.last_running_child = 0
        for child in self.children:
            child.setup()

    def _do_tick(self):
        if not self.children:
            self.logwarn("Ticking without children. Is this really what you want?")
            return NodeMsg.SUCCEEDED

        # If we've previously succeeded or failed, reset
        # last_running_child and untick all children
        if self.state in [NodeMsg.SUCCEEDED, NodeMsg.FAILED]:
            self.last_running_child = 0
            for child in self.children:
                child.reset()

        # Tick children until one returns FAILED or RUNNING
        for index, child in enumerate(self.children):
            if index < self.last_running_child:
                continue
            result = child.tick()

            if result != NodeMsg.SUCCEEDED:
                if result == NodeMsg.RUNNING:
                    self.last_running_child = index
                else:
                    # For all states other than RUNNING, untick all
                    # children after the one that hasn't succeeded
                    for untick_child in self.children[index + 1 :]:
                        untick_child.untick()
                return result
        # If all children succeeded, we too succeed
        return NodeMsg.SUCCEEDED

    def _do_untick(self):
        for child in self.children:
            child.untick()
        self.last_running_child = 0
        return NodeMsg.IDLE

    def _do_reset(self):
        for child in self.children:
            child.reset()
        self.last_running_child = 0
        return NodeMsg.IDLE

    def _do_shutdown(self):
        for child in self.children:
            child.shutdown()
        self.last_running_child = 0

    def _do_calculate_utility(self):
        return calculate_utility_sequence(self.children)


def calculate_utility_sequence(children):
    """Shared Utility aggregation for Sequence and MemorySequence."""
    bounds = UtilityBounds(
        can_execute=True,
        has_lower_bound_success=True,
        has_upper_bound_success=True,
        has_lower_bound_failure=True,
        has_upper_bound_failure=True,
    )
    if children:
        # These are the direct inverse of the Fallback's boundaries - check
        # the detailed description in fallback.py if you're interested in
        # why this is the solution.
        failure_bounds = [
            UtilityBounds(
                has_lower_bound_failure=True,
                lower_bound_failure=0,
                has_upper_bound_failure=True,
                upper_bound_failure=0,
            )
            for _ in children
        ]
        for index, child_bounds in enumerate(
            (child.calculate_utility() for child in children)
        ):
            # If any child cannot execute at all, the sequence cannot
            # execute and won't provide a utility estimate:
            if not child_bounds.can_execute:
                return UtilityBounds()

            bounds.has_lower_bound_success &= child_bounds.has_lower_bound_success
            bounds.lower_bound_success += child_bounds.lower_bound_success
            bounds.has_upper_bound_success &= child_bounds.has_upper_bound_success
            bounds.upper_bound_success += child_bounds.upper_bound_success

            failure_bounds[
                index
            ].lower_bound_failure += child_bounds.lower_bound_failure
            failure_bounds[
                index
            ].has_lower_bound_failure &= child_bounds.has_lower_bound_failure
            failure_bounds[
                index
            ].upper_bound_failure += child_bounds.upper_bound_failure
            failure_bounds[
                index
            ].has_upper_bound_failure &= child_bounds.has_upper_bound_failure
            # Range returns an empty range if the first parameter is larger
            # than the second, so no bounds checking necessary
            for i in range(index + 1, len(failure_bounds)):
                failure_bounds[
                    i
                ].lower_bound_failure += child_bounds.lower_bound_success
                failure_bounds[
                    i
                ].has_lower_bound_failure &= child_bounds.has_lower_bound_success
                failure_bounds[
                    i
                ].upper_bound_failure += child_bounds.upper_bound_success
                failure_bounds[
                    i
                ].has_upper_bound_failure &= child_bounds.has_upper_bound_success

        # Select the minimum and maximum values to get the final bounds
        bounds.lower_bound_failure = min(
            (x.lower_bound_failure for x in failure_bounds)
        )
        bounds.has_lower_bound_failure &= all(
            (b.has_lower_bound_failure for b in failure_bounds)
        )

        bounds.upper_bound_failure = max(
            (x.upper_bound_failure for x in failure_bounds)
        )
        bounds.has_upper_bound_failure &= all(
            (b.has_upper_bound_failure for b in failure_bounds)
        )

    return bounds
