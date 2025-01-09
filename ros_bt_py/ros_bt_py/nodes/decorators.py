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
from result import Result, Ok, Err, is_err

from ros_bt_py_interfaces.msg import UtilityBounds

from ros_bt_py.node import Decorator, define_bt_node
from ros_bt_py.node_config import NodeConfig
from ros_bt_py.helpers import BTNodeState


@define_bt_node(
    NodeConfig(version="0.1.0", options={}, inputs={}, outputs={}, max_children=1)
)
class IgnoreFailure(Decorator):
    """
    Return SUCCEEDED regardless of whether the child actually succeeded.

    RUNNING is forwarded

    """

    def _do_setup(self):
        for child in self.children:
            return child.setup()
        return Ok(BTNodeState.IDLE)

    def _do_tick(self):
        for child in self.children:
            result = child.tick()
            if result.is_err():
                return result
            if result.unwrap() == BTNodeState.FAILED:
                return Ok(BTNodeState.SUCCEEDED)
            return result

        # Succeed if we have no children
        return Ok(BTNodeState.SUCCEEDED)

    def _do_shutdown(self):
        for child in self.children:
            return child.shutdown()
        return Ok(BTNodeState.SHUTDOWN)

    def _do_reset(self):
        for child in self.children:
            return child.reset()
        return Ok(BTNodeState.IDLE)

    def _do_untick(self):
        for child in self.children:
            return child.untick()
        return Ok(BTNodeState.IDLE)


@define_bt_node(
    NodeConfig(
        version="0.1.0",
        options={"running_is_success": bool},
        inputs={},
        outputs={},
        max_children=1,
    )
)
class IgnoreRunning(Decorator):
    """Return SUCCESS or FAILURE when the child returns RUNNING."""

    def _do_setup(self):
        for child in self.children:
            return child.setup()
        return Ok(BTNodeState.IDLE)

    def _do_tick(self):
        for child in self.children:
            result = child.tick()
            if result.is_err():
                return result

            if result.unwrap() == BTNodeState.RUNNING:
                if self.options["running_is_success"]:
                    return Ok(BTNodeState.SUCCEEDED)
                else:
                    return Ok(BTNodeState.FAILED)
            return result

        # Fails if we have no children
        return Ok(BTNodeState.FAILED)

    def _do_shutdown(self):
        for child in self.children:
            return child.shutdown()
        return Ok(BTNodeState.SHUTDOWN)

    def _do_reset(self):
        for child in self.children:
            return child.reset()
        return Ok(BTNodeState.IDLE)

    def _do_untick(self):
        for child in self.children:
            return child.untick()
        return Ok(BTNodeState.IDLE)


@define_bt_node(
    NodeConfig(version="0.1.0", options={}, inputs={}, outputs={}, max_children=1)
)
class IgnoreSuccess(Decorator):
    """
    Return FAILURE regardless of whether the child actually failed.

    RUNNING is forwarded

    """

    def _do_setup(self):
        for child in self.children:
            return child.setup()
        return Ok(BTNodeState.IDLE)

    def _do_tick(self):
        for child in self.children:
            result = child.tick()
            if result.is_err():
                return result
            if result.unwrap() == BTNodeState.SUCCEEDED:
                return Ok(BTNodeState.FAILED)
            return result

        # Fails if we have no children
        return Ok(BTNodeState.FAILED)

    def _do_shutdown(self):
        for child in self.children:
            return child.shutdown()
        return Ok(BTNodeState.SHUTDOWN)

    def _do_reset(self):
        for child in self.children:
            return child.reset()
        return Ok(BTNodeState.IDLE)

    def _do_untick(self):
        for child in self.children:
            return child.untick()
        return Ok(BTNodeState.IDLE)


@define_bt_node(
    NodeConfig(version="0.1.0", options={}, inputs={}, outputs={}, max_children=1)
)
class UntilSuccess(Decorator):
    """
    Return RUNNING until the child node returns SUCCEEDED.

    This means the child node will be retried until it succeeds or the
    tree moves on to a different branch. Failure means a restart and
    will be translated into RUNNING!

    """

    def _do_setup(self):
        for child in self.children:
            return child.setup()
        return Ok(BTNodeState.IDLE)

    def _do_tick(self):
        for child in self.children:
            result = child.tick()
            if result.is_err():
                return result
            if result.unwrap() == BTNodeState.FAILED:
                return Ok(BTNodeState.RUNNING)
            return result

        # Succeed if we have no children
        return Ok(BTNodeState.SUCCEEDED)

    def _do_shutdown(self):
        for child in self.children:
            return child.shutdown()
        return Ok(BTNodeState.SHUTDOWN)

    def _do_reset(self):
        for child in self.children:
            return child.reset()
        return Ok(BTNodeState.IDLE)

    def _do_untick(self):
        for child in self.children:
            return child.untick()
        return Ok(BTNodeState.IDLE)


@define_bt_node(
    NodeConfig(version="0.1.0", options={}, inputs={}, outputs={}, max_children=1)
)
class Inverter(Decorator):
    """
    Inverts the result of the child.

    Return SUCCEEDED if the child returned FAILED,
    return FAILED if the child returned SUCCEEDED.

    RUNNING is forwarded
    """

    def _do_setup(self):
        for child in self.children:
            return child.setup()
        return Ok(BTNodeState.IDLE)

    def _do_tick(self):
        for child in self.children:
            result = child.tick()
            if result.is_err():
                return result

            if result.value == BTNodeState.FAILED:
                return Ok(BTNodeState.SUCCEEDED)
            elif result.value == BTNodeState.SUCCEEDED:
                return Ok(BTNodeState.FAILED)
            return result

        # Succeed if we have no children
        return Ok(BTNodeState.SUCCEEDED)

    def _do_shutdown(self):
        for child in self.children:
            return child.shutdown()
        return Ok(BTNodeState.SHUTDOWN)

    def _do_reset(self):
        for child in self.children:
            return child.reset()
        return Ok(BTNodeState.IDLE)

    def _do_untick(self):
        for child in self.children:
            return child.untick()
        return Ok(BTNodeState.IDLE)


@define_bt_node(
    NodeConfig(
        version="0.1.0",
        options={"num_retries": int},
        inputs={},
        outputs={},
        max_children=1,
    )
)
class Retry(Decorator):
    """
    Retry the child `num_retries` times.

    Retry, here, means ignoring a FAILED result from the child,
    reporting it as RUNNING instead and resetting it.

    """

    def _do_setup(self):
        self._retry_count = 0
        for child in self.children:
            return child.setup()
        return Ok(BTNodeState.IDLE)

    def _do_tick(self):
        for child in self.children:
            result = child.tick()

            if result.is_ok() and result.value == BTNodeState.FAILED:
                if self._retry_count < self.options["num_retries"]:
                    self._retry_count += 1

                    reset_result = child.reset()
                    if reset_result.is_err():
                        return reset_result

                    return Ok(BTNodeState.RUNNING)
                else:
                    self._retry_count = 0
                    return Ok(BTNodeState.FAILED)
            return result

        # Succeed if we have no children
        return Ok(BTNodeState.SUCCEEDED)

    def _do_shutdown(self):
        for child in self.children:
            return child.shutdown()
        return Ok(BTNodeState.SHUTDOWN)

    def _do_reset(self):
        self._retry_count = 0
        for child in self.children:
            return child.reset()
        return Ok(BTNodeState.IDLE)

    def _do_untick(self):
        for child in self.children:
            return child.untick()
        return Ok(BTNodeState.IDLE)


@define_bt_node(
    NodeConfig(
        version="0.1.0",
        options={"num_repeats": int},
        inputs={},
        outputs={},
        max_children=1,
    )
)
class Repeat(Decorator):
    """
    Repeat the child `num_repeat` times.

    Repeat, here, means counting the number of times the child SUCCEEDED,
    if the number of repeats is not yet reached, the child will be resetted.
    Returns RUNNING while the number of repeats is not yet reached,
    returns FAILED when the child fails.

    """

    def _do_setup(self):
        self._repeat_count = 0
        for child in self.children:
            return child.setup()
        return Ok(BTNodeState.IDLE)

    def _do_tick(self):
        for child in self.children:
            result = child.tick()
            if result.is_err():
                return result

            if result.value == BTNodeState.FAILED:
                return Ok(BTNodeState.FAILED)
            elif result.value == BTNodeState.SUCCEEDED:
                if self._repeat_count < self.options["num_repeats"]:
                    self._repeat_count += 1
                    reset_result = child.reset()
                    if reset_result.is_err():
                        return reset_result
                    return Ok(BTNodeState.RUNNING)
                else:
                    return Ok(BTNodeState.SUCCEEDED)
            return result

        # Succeed if we have no children
        return Ok(BTNodeState.SUCCEEDED)

    def _do_shutdown(self):
        for child in self.children:
            return child.shutdown()
        return Ok(BTNodeState.SHUTDOWN)

    def _do_reset(self):
        self._repeat_count = 0
        for child in self.children:
            return child.reset()
        return Ok(BTNodeState.IDLE)

    def _do_untick(self):
        for child in self.children:
            return child.untick()
        return Ok(BTNodeState.IDLE)


@define_bt_node(
    NodeConfig(
        version="0.1.0",
        options={},
        inputs={"reset": bool},
        outputs={},
        optional_options=["reset"],
        max_children=1,
    )
)
class RepeatNoAutoReset(Repeat):
    """
    Repeat the child `num_repeat` times but does not reset.

    Repeat, here, means counting the number of times the child SUCCEEDED,
    if the number of repeats is not yet reached, the child will be resetted.
    Returns RUNNING while the number of repeats is not yet reached,
    returns FAILED when the child fails.

    This Repeat will NOT reset its count when _do_reset() is called which means
    it will not reset once the tree is running, other than by deliberate decission within
    the tree. This can be used to "reset" the execution of parts of the tree if certain conditions
    have been met.
    """

    def _do_setup(self):
        self._received_in = False
        return super(RepeatNoAutoReset, self)._do_setup()

    def _do_tick(self):
        if self.inputs["reset"] is not None and self.inputs["reset"]:
            self._repeat_count = 0
            self.inputs["reset"] = False

        # Only TICK the children
        if self._repeat_count < self.options["num_repeats"]:
            for child in self.children:
                result = child.tick()
                if result.is_err():
                    return result
                if result.value == BTNodeState.FAILED:
                    return Ok(BTNodeState.FAILED)
                elif result.value == BTNodeState.SUCCEEDED:
                    if self._repeat_count < self.options["num_repeats"]:
                        self._repeat_count += 1
                        reset_result = child.reset()
                        if reset_result.is_err():
                            return reset_result
                        return Ok(BTNodeState.RUNNING)
                    else:
                        return Ok(BTNodeState.SUCCEEDED)
                return result

        # Succeed if we have no children
        return Ok(BTNodeState.SUCCEEDED)

    def _do_reset(self):
        self._received_in = False
        # Only reset childs if we havent reached our goal
        if self._repeat_count < self.options["num_repeats"]:
            for child in self.children:
                return child.reset()
        return Ok(BTNodeState.IDLE)


@define_bt_node(
    NodeConfig(version="0.1.0", options={}, inputs={}, outputs={}, max_children=1)
)
class RepeatAlways(Decorator):
    """
    Repeats the child an infinite number of times.

    returns RUNNING if there is a child, regardless of the childs returned state,

    returns SUCCEEDED if there is no child.

    """

    def _do_setup(self):
        for child in self.children:
            return child.setup()
        return Ok(BTNodeState.IDLE)

    def _do_tick(self):
        for child in self.children:
            result = child.tick()
            if result.is_err():
                return result
            if result.value != BTNodeState.RUNNING:
                reset_result = child.reset()
                if reset_result.is_err():
                    return reset_result
            return Ok(BTNodeState.RUNNING)

        # Succeed if we have no children
        return Ok(BTNodeState.SUCCEEDED)

    def _do_shutdown(self):
        for child in self.children:
            return child.shutdown()
        return Ok(BTNodeState.SHUTDOWN)

    def _do_reset(self):
        for child in self.children:
            return child.reset()
        return Ok(BTNodeState.IDLE)

    def _do_untick(self):
        for child in self.children:
            return child.untick()
        return Ok(BTNodeState.IDLE)


@define_bt_node(
    NodeConfig(version="0.1.0", options={}, inputs={}, outputs={}, max_children=1)
)
class RepeatUntilFail(Decorator):
    """
    Repeats the child an infinite number of times until it returns FAILED.

    returns RUNNING if the child is RUNNING or has returned SUCCEEDED,
    returns FAILED if the child returned FAILED

    returns SUCCEEDED if there is no child.

    """

    def _do_setup(self):
        for child in self.children:
            return child.setup()
        return Ok(BTNodeState.IDLE)

    def _do_tick(self):
        for child in self.children:
            result = child.tick()
            if result.is_err():
                return result
            if result.value == BTNodeState.FAILED:
                return Ok(BTNodeState.FAILED)
            elif result.value == BTNodeState.SUCCEEDED:
                reset_result = child.reset()
                if reset_result.is_err():
                    return reset_result
            return Ok(BTNodeState.RUNNING)

        # Succeed if we have no children
        return BTNodeState.SUCCEEDED

    def _do_shutdown(self):
        for child in self.children:
            return child.shutdown()
        return Ok(BTNodeState.SHUTDOWN)

    def _do_reset(self):
        for child in self.children:
            return child.reset()
        return Ok(BTNodeState.IDLE)

    def _do_untick(self):
        for child in self.children:
            return child.untick()
        return Ok(BTNodeState.IDLE)


@define_bt_node(
    NodeConfig(version="0.1.0", options={}, inputs={}, outputs={}, max_children=1)
)
class RepeatIfFail(Decorator):
    """
    Repeats the child an infinite number of times if it returns FAILED.

    returns RUNNING if the child is RUNNING,
    returns FAILED if the child returned FAILED

    returns SUCCEEDED if the child SUCCEEDED or if there is no child.

    """

    def _do_setup(self):
        for child in self.children:
            return child.setup()
        return Ok(BTNodeState.IDLE)

    def _do_tick(self):
        for child in self.children:
            result = child.tick()
            if result.is_err():
                return result
            if result == BTNodeState.FAILED:
                reset_result = child.reset()
                if reset_result.is_err():
                    return reset_result
            elif result.value == BTNodeState.SUCCEEDED:
                return Ok(BTNodeState.SUCCEEDED)
            return BTNodeState.RUNNING

        # Succeed if we have no children
        return Ok(BTNodeState.SUCCEEDED)

    def _do_shutdown(self):
        for child in self.children:
            return child.shutdown()
        return Ok(BTNodeState.SHUTDOWN)

    def _do_reset(self):
        for child in self.children:
            return child.reset()
        return Ok(BTNodeState.IDLE)

    def _do_untick(self):
        for child in self.children:
            return child.untick()
        return Ok(BTNodeState.IDLE)


@define_bt_node(
    NodeConfig(version="0.1.0", options={}, inputs={}, outputs={}, max_children=1)
)
class Optional(Decorator):
    """
    Wraps a child that may not be able to execute.

    A child that cannot execute will not be ticked. Instead, this
    decorator will always return SUCCEEDED.

    If the child *can* execute, the decorator will simply forward all
    events.

    """

    def _do_setup(self):
        self.execute_child = False
        for child in self.children:
            utility = child.calculate_utility()
            if utility.is_ok():
                utility_bounds: UtilityBounds = utility.ok()
                if utility_bounds.can_execute:
                    self.execute_child = True
                    return child.setup()
            else:
                return Err(utility.err())
        return Ok(BTNodeState.IDLE)

    def _do_tick(self):
        if self.execute_child:
            return self.children[0].tick()
        else:
            return Ok(BTNodeState.SUCCEEDED)

    def _do_shutdown(self):
        if self.execute_child:
            return self.children[0].shutdown()
        return Ok(BTNodeState.SHUTDOWN)

    def _do_reset(self):
        if self.execute_child:
            return self.children[0].reset()
        else:
            return Ok(BTNodeState.IDLE)

    def _do_untick(self):
        if self.execute_child:
            return self.children[0].untick()
        else:
            return Ok(BTNodeState.IDLE)

    def _do_calculate_utility(self):
        for child in self.children:
            bounds = child.calculate_utility()
            if bounds.is_err():
                return bounds
            if bounds.value.can_execute:
                return bounds

        # If the child can't execute, return a UtilityBounds object
        # that can execute, but does not have any bounds set (that is,
        # if another executor can actually execute our child, it is
        # pretty much guaranteed to have a better Utility score than
        # us)
        return Ok(UtilityBounds(can_execute=True))


@define_bt_node(
    NodeConfig(
        version="0.1.0", options={}, inputs={"watch": str}, outputs={}, max_children=1
    )
)
class Watch(Decorator):
    """Untick child if watch string changed."""

    def _do_setup(self):
        self.previous_watch = float("NaN")
        for child in self.children:
            return child.setup()
        return Ok(BTNodeState.IDLE)

    def _do_tick(self):
        if len(self.children) == 0:
            return Ok(BTNodeState.SUCCEEDED)

        child = self.children[0]
        if self.previous_watch != self.inputs["watch"]:
            untick_result = child.untick()
            if untick_result.is_err():
                return untick_result
            self.previous_watch = self.inputs["watch"]

        return self.children[0].tick()

    def _do_shutdown(self):
        self.previous_watch = float("NaN")
        for child in self.children:
            return child.shutdown()
        return Ok(BTNodeState.SHUTDOWN)

    def _do_reset(self):
        self.previous_watch = float("NaN")
        for child in self.children:
            return child.reset()
        return Ok(BTNodeState.IDLE)

    def _do_untick(self):
        self.previous_watch = float("NaN")
        for child in self.children:
            return child.untick()
        return Ok(BTNodeState.IDLE)
