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
import math

from result import Ok, Err, is_err

from ros_bt_py_interfaces.msg import UtilityBounds

from ros_bt_py.exceptions import BehaviorTreeException
from ros_bt_py.node import FlowControl, define_bt_node, BTNodeState
from ros_bt_py.node_config import NodeConfig


@define_bt_node(
    NodeConfig(
        version="0.1.0",
        options={"needed_successes": int},
        inputs={},
        outputs={},
        max_children=None,
    )
)
class Parallel(FlowControl):
    """
    The Parallel ticks all of its children every tick.

    Its success is determined depending on `needed_successes` - if at
    least `needed_success` of its children have returned SUCCEEDED,
    the Parallel too returns SUCCEEDED.

    On the other hand, if more than N - `needed_successes` of its
    children have returned FAILED, so will the Parallel.

    Different values of `needed_successes` can produce behavior
    similar to the "barrier" (wait until all actions are finished) and
    "heureka" (first success aborts all other actions) parallel
    programming primitives, and more.

    To prevent thrashing when multiple children return RUNNING for
    different lengths of time, a child that has returned SUCCEEDED or
    FAILED once will not be ticked again untill **all** children have
    reached a result.

    """

    def _do_setup(self):
        if len(self.children) < self.options["needed_successes"]:
            return Err(
                BehaviorTreeException(
                    f"Option value needed_successes ({self.options['needed_successes']})"
                    " cannot be larger than "
                    f"the number of children ({len(self.children)})"
                )
            )
        for child in self.children:
            result = child.setup()
            if result.is_err():
                return result
        return Ok(BTNodeState.IDLE)

    def _do_tick(self):
        # Just like Sequence and Fallback, reset after having returned
        # SUCCEEDED or FAILED once
        if self.state in [BTNodeState.SUCCEEDED, BTNodeState.FAILED]:
            for child in self.children:
                result = child.reset()
                if result.is_err():
                    return result

        successes = 0
        failures = 0
        for child in self.children:
            if child.state not in [BTNodeState.SUCCEEDED, BTNodeState.FAILED]:
                result = child.tick()
                if result.is_err():
                    return result
            if child.state == BTNodeState.SUCCEEDED:
                successes += 1
            if child.state == BTNodeState.FAILED:
                failures += 1
        if successes >= self.options["needed_successes"]:
            # untick all running children
            for child in self.children:
                if child.state not in [BTNodeState.SUCCEEDED, BTNodeState.FAILED]:
                    result = child.untick()
                    if result.is_err():
                        return result
            return Ok(BTNodeState.SUCCEEDED)
        elif failures > len(self.children) - self.options["needed_successes"]:
            # untick all running children
            for child in self.children:
                if child.state not in [BTNodeState.SUCCEEDED, BTNodeState.FAILED]:
                    result = child.untick()
                    if result.is_err():
                        return result
            return Ok(BTNodeState.FAILED)
        return Ok(BTNodeState.RUNNING)

    def _do_shutdown(self):
        for child in self.children:
            result = child.shutdown()
            if result.is_err():
                return result
        return Ok(BTNodeState.SHUTDOWN)

    def _do_reset(self):
        for child in self.children:
            result = child.reset()
            if result.is_err():
                return result
        return Ok(BTNodeState.IDLE)

    def _do_untick(self):
        for child in self.children:
            result = child.untick()
            if result.is_err():
                return result
        return Ok(BTNodeState.IDLE)

    def _do_calculate_utility(self):
        if len(self.children) < self.options["needed_successes"]:
            return Err(
                BehaviorTreeException(
                    f"Option value needed_successes ({self.options['needed_successes']}) "
                    " cannot be larger than "
                    f"the number of children ({len(self.children)})"
                )
            )

        # This calculation is fairly straightforward:
        #
        # The lower bound for success is the lowest sum of
        # "needed_successes" children's lower success bounds.
        #
        # The upper bound for success is the highest sum of
        # "needed_successes" children's upper success bounds.
        #
        # We get both by first sorting the array of bounds by the
        # respective bound (ascending for the lower bounds, descending
        # for the upper bounds) and then summing the first
        # "needed_successes" values from the sorted array.

        child_bounds_results = [child.calculate_utility() for child in self.children]
        bounds = UtilityBounds()
        failures = [
            child_bound.err()
            for child_bound in child_bounds_results
            if child_bound.is_err()
        ]

        if len(failures) > 0:
            return Err(
                BehaviorTreeException(
                    f"Could not calculate utility for all child nodes: {failures}"
                )
            )

        child_bounds = [
            child_bound.ok()
            for child_bound in child_bounds_results
            if child_bound.is_ok()
        ]
        bounds.can_execute = all((b.can_execute for b in child_bounds))

        # Early return if any child cannot execute
        if not bounds.can_execute:
            return Ok(bounds)

        success_threshold = self.options["needed_successes"]

        best_success_group = sorted(child_bounds, key=lambda b: b.lower_bound_success)[
            :success_threshold
        ]
        bounds.has_lower_bound_success = all(
            (b.has_lower_bound_success for b in best_success_group)
        )
        if bounds.has_lower_bound_success:
            bounds.lower_bound_success = math.fsum(
                (b.lower_bound_success for b in best_success_group)
            )

        worst_success_group = sorted(
            child_bounds, reverse=True, key=lambda b: b.upper_bound_success
        )[:success_threshold]
        bounds.has_upper_bound_success = all(
            (b.has_upper_bound_success for b in worst_success_group)
        )
        if bounds.has_upper_bound_success:
            bounds.upper_bound_success = math.fsum(
                (b.upper_bound_success for b in worst_success_group)
            )

        # Now we do the same for the failures - the only difference
        # here is that failure_threshold is
        #
        # 1 + (num_children - success_threshold)
        #
        # because of the semantics of Parallel explained in the class
        # docstring

        # The minimum number of child failures for an overall Parallel failure
        failure_threshold = 1 + (len(self.children) - success_threshold)

        best_failure_group = sorted(child_bounds, key=lambda b: b.lower_bound_failure)[
            :failure_threshold
        ]
        bounds.has_lower_bound_failure = all(
            (b.has_lower_bound_failure for b in best_failure_group)
        )
        if bounds.has_lower_bound_failure:
            bounds.lower_bound_failure = math.fsum(
                (b.lower_bound_failure for b in best_failure_group)
            )

        worst_failure_group = sorted(
            child_bounds, reverse=True, key=lambda b: b.upper_bound_failure
        )[:failure_threshold]
        bounds.has_upper_bound_failure = all(
            (b.has_upper_bound_failure for b in worst_failure_group)
        )
        if bounds.has_upper_bound_failure:
            bounds.upper_bound_failure = math.fsum(
                (b.upper_bound_failure for b in worst_failure_group)
            )

        return Ok(bounds)


@define_bt_node(
    NodeConfig(
        version="0.1.0",
        options={"needed_successes": int, "tolerate_failures": int},
        inputs={},
        outputs={},
        max_children=None,
    )
)
class ParallelFailureTolerance(FlowControl):
    """
    The ParallelFailureTolerance ticks all of its children every tick.

    Its success is determined depending on `needed_successes` - if at
    least `needed_success` of its children have returned SUCCEEDED,
    the Parallel too returns SUCCEEDED.

    On the other hand, if more than `tolerate_failures` of its
    children have returned FAILED, so will the ParallelFailureTolerance.
    This extra parameter is the only difference with the Parallel node.

    To prevent thrashing when multiple children return RUNNING for
    different lengths of time, a child that has returned SUCCEEDED or
    FAILED once will not be ticked again untill **all** children have
    reached a result.

    """

    def _do_setup(self):
        if len(self.children) < self.options["needed_successes"]:
            return Err(
                BehaviorTreeException(
                    f"Option value needed_successes ({self.options['needed_successes']})"
                    " cannot be larger than "
                    f"the number of children ({len(self.children)})"
                )
            )
        for child in self.children:
            result = child.setup()
            if result.is_err():
                return result
        return Ok(BTNodeState.IDLE)

    def _do_tick(self):
        # Just like Sequence and Fallback, reset after having returned
        # SUCCEEDED or FAILED once
        if self.state in [BTNodeState.SUCCEEDED, BTNodeState.FAILED]:
            for child in self.children:
                result = child.reset()
                if result.is_err():
                    return result

        successes = 0
        failures = 0
        for child in self.children:
            if child.state not in [BTNodeState.SUCCEEDED, BTNodeState.FAILED]:
                result = child.tick()
                if result.is_err():
                    return result
            if child.state == BTNodeState.SUCCEEDED:
                successes += 1
            if child.state == BTNodeState.FAILED:
                failures += 1
        if successes >= self.options["needed_successes"]:
            # untick all running children
            for child in self.children:
                if child.state not in [BTNodeState.SUCCEEDED, BTNodeState.FAILED]:
                    result = child.untick()
                    if result.is_err():
                        return result
            return Ok(BTNodeState.SUCCEEDED)
        elif failures > self.options["tolerate_failures"]:
            # untick all running children
            for child in self.children:
                if child.state not in [BTNodeState.SUCCEEDED, BTNodeState.FAILED]:
                    result = child.untick()
                    if result.is_err():
                        return result
            return Ok(BTNodeState.FAILED)
        return Ok(BTNodeState.RUNNING)

    def _do_shutdown(self):
        for child in self.children:
            result = child.shutdown()
            if result.is_err():
                return result
        return Ok(BTNodeState.SHUTDOWN)

    def _do_reset(self):
        for child in self.children:
            result = child.reset()
            if result.is_err():
                return result
        return Ok(BTNodeState.IDLE)

    def _do_untick(self):
        for child in self.children:
            result = child.untick()
            if result.is_err():
                return result
        return Ok(BTNodeState.IDLE)

    def _do_calculate_utility(self):
        if len(self.children) < self.options["needed_successes"]:
            return Err(
                BehaviorTreeException(
                    f"Option value needed_successes ({self.options['needed_successes']})"
                    " cannot be larger than "
                    f"the number of children ({len(self.children)})"
                )
            )

        # This calculation is fairly straightforward:
        #
        # The lower bound for success is the lowest sum of
        # "needed_successes" children's lower success bounds.
        #
        # The upper bound for success is the highest sum of
        # "needed_successes" children's upper success bounds.
        #
        # We get both by first sorting the array of bounds by the
        # respective bound (ascending for the lower bounds, descending
        # for the upper bounds) and then summing the first
        # "needed_successes" values from the sorted array.
        child_bounds_results = [child.calculate_utility() for child in self.children]
        bounds = UtilityBounds()
        failures = [
            child_bound.err()
            for child_bound in child_bounds_results
            if child_bound.is_err()
        ]

        if len(failures) > 0:
            return Err(
                BehaviorTreeException(
                    "Could not calculate utility for all child nodes:" f" {failures}"
                )
            )

        child_bounds = [
            child_bound.ok()
            for child_bound in child_bounds_results
            if child_bound.is_ok()
        ]
        bounds.can_execute = all((b.can_execute for b in child_bounds))

        # Early return if any child cannot execute
        if not bounds.can_execute:
            return Ok(bounds)

        success_threshold = self.options["needed_successes"]

        best_success_group = sorted(child_bounds, key=lambda b: b.lower_bound_success)[
            :success_threshold
        ]
        bounds.has_lower_bound_success = all(
            (b.has_lower_bound_success for b in best_success_group)
        )
        if bounds.has_lower_bound_success:
            bounds.lower_bound_success = math.fsum(
                (b.lower_bound_success for b in best_success_group)
            )

        worst_success_group = sorted(
            child_bounds, reverse=True, key=lambda b: b.upper_bound_success
        )[:success_threshold]
        bounds.has_upper_bound_success = all(
            (b.has_upper_bound_success for b in worst_success_group)
        )
        if bounds.has_upper_bound_success:
            bounds.upper_bound_success = math.fsum(
                (b.upper_bound_success for b in worst_success_group)
            )

        # Now we do the same for the failures - the only difference
        # here is that failure_threshold is
        #
        # 1 + (num_children - success_threshold)
        #
        # because of the semantics of Parallel explained in the class
        # docstring

        # The minimum number of child failures for an overall Parallel failure
        failure_threshold = 1 + (len(self.children) - success_threshold)

        best_failure_group = sorted(child_bounds, key=lambda b: b.lower_bound_failure)[
            :failure_threshold
        ]
        bounds.has_lower_bound_failure = all(
            (b.has_lower_bound_failure for b in best_failure_group)
        )
        if bounds.has_lower_bound_failure:
            bounds.lower_bound_failure = math.fsum(
                (b.lower_bound_failure for b in best_failure_group)
            )

        worst_failure_group = sorted(
            child_bounds, reverse=True, key=lambda b: b.upper_bound_failure
        )[:failure_threshold]
        bounds.has_upper_bound_failure = all(
            (b.has_upper_bound_failure for b in worst_failure_group)
        )
        if bounds.has_upper_bound_failure:
            bounds.upper_bound_failure = math.fsum(
                (b.upper_bound_failure for b in worst_failure_group)
            )

        return Ok(bounds)
