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

from ros_bt_py_interfaces.msg import UtilityBounds

from ros_bt_py.data_types import BoolType, IntType, StringType
from ros_bt_py.node import Decorator, define_bt_node
from ros_bt_py.node_config import NodeConfig
from ros_bt_py.helpers import BTNodeState
from ros_bt_py.exceptions import BehaviorTreeException


@define_bt_node(NodeConfig(inputs={}, outputs={}, max_children=1))
class IgnoreFailure(Decorator):
    """
    Return SUCCEEDED regardless of whether the child actually succeeded.

    RUNNING is forwarded

    """

    def _do_setup(self) -> Result[BTNodeState, BehaviorTreeException]:
        for child in self.children:
            return child.setup()
        return Ok(BTNodeState.IDLE)

    def _do_tick(self) -> Result[BTNodeState, BehaviorTreeException]:
        for child in self.children:
            return child.tick().map(
                lambda state: (
                    BTNodeState.SUCCEEDED if state == BTNodeState.FAILED else state
                )
            )

        # Succeed if we have no children
        return Ok(BTNodeState.SUCCEEDED)

    def _do_shutdown(self) -> Result[BTNodeState, BehaviorTreeException]:
        return Ok(BTNodeState.SHUTDOWN)

    def _do_reset(self) -> Result[BTNodeState, BehaviorTreeException]:
        for child in self.children:
            return child.reset()
        return Ok(BTNodeState.IDLE)

    def _do_untick(self) -> Result[BTNodeState, BehaviorTreeException]:
        for child in self.children:
            return child.untick()
        return Ok(BTNodeState.IDLE)


@define_bt_node(
    NodeConfig(
        inputs={"running_is_success": BoolType(allow_dynamic=False)},
        outputs={},
        max_children=1,
    )
)
class IgnoreRunning(Decorator):
    """Return SUCCESS or FAILURE when the child returns RUNNING."""

    def _do_setup(self) -> Result[BTNodeState, BehaviorTreeException]:
        for child in self.children:
            return child.setup()
        match self.inputs.get_value_as("running_is_success", bool):
            case Err(e):
                return Err(e)
            case Ok(b):
                self._running_is_success = b
        return Ok(BTNodeState.IDLE)

    def _do_tick(self) -> Result[BTNodeState, BehaviorTreeException]:
        for child in self.children:
            return child.tick().map(
                lambda state: (
                    state
                    if state != BTNodeState.RUNNING
                    else (
                        BTNodeState.SUCCEEDED
                        if self._running_is_success
                        else BTNodeState.FAILED
                    )
                )
            )

        # Fails if we have no children
        return Ok(BTNodeState.FAILED)

    def _do_shutdown(self) -> Result[BTNodeState, BehaviorTreeException]:
        return Ok(BTNodeState.SHUTDOWN)

    def _do_reset(self) -> Result[BTNodeState, BehaviorTreeException]:
        for child in self.children:
            return child.reset()
        return Ok(BTNodeState.IDLE)

    def _do_untick(self) -> Result[BTNodeState, BehaviorTreeException]:
        for child in self.children:
            return child.untick()
        return Ok(BTNodeState.IDLE)


@define_bt_node(NodeConfig(inputs={}, outputs={}, max_children=1))
class IgnoreSuccess(Decorator):
    """
    Return FAILURE regardless of whether the child actually failed.

    RUNNING is forwarded

    """

    def _do_setup(self) -> Result[BTNodeState, BehaviorTreeException]:
        for child in self.children:
            return child.setup()
        return Ok(BTNodeState.IDLE)

    def _do_tick(self) -> Result[BTNodeState, BehaviorTreeException]:
        for child in self.children:
            return child.tick().map(
                lambda state: (
                    BTNodeState.FAILED if state == BTNodeState.SUCCEEDED else state
                )
            )

        # Fails if we have no children
        return Ok(BTNodeState.FAILED)

    def _do_shutdown(self) -> Result[BTNodeState, BehaviorTreeException]:
        return Ok(BTNodeState.SHUTDOWN)

    def _do_reset(self) -> Result[BTNodeState, BehaviorTreeException]:
        for child in self.children:
            return child.reset()
        return Ok(BTNodeState.IDLE)

    def _do_untick(self) -> Result[BTNodeState, BehaviorTreeException]:
        for child in self.children:
            return child.untick()
        return Ok(BTNodeState.IDLE)


@define_bt_node(NodeConfig(inputs={}, outputs={}, max_children=1))
class UntilSuccess(Decorator):
    """
    Return RUNNING until the child node returns SUCCEEDED.

    This means the child node will be retried until it succeeds or the
    tree moves on to a different branch. Failure means a restart and
    will be translated into RUNNING!

    """

    def _do_setup(self) -> Result[BTNodeState, BehaviorTreeException]:
        for child in self.children:
            return child.setup()
        return Ok(BTNodeState.IDLE)

    def _do_tick(self) -> Result[BTNodeState, BehaviorTreeException]:
        for child in self.children:
            match child.tick():
                case Err(e):
                    return Err(e)
                case Ok(s):
                    state = s
            if state == BTNodeState.FAILED:
                return child.reset().map(lambda _: BTNodeState.RUNNING)
            return Ok(state)
        # Succeed if we have no children
        return Ok(BTNodeState.SUCCEEDED)

    def _do_shutdown(self) -> Result[BTNodeState, BehaviorTreeException]:
        return Ok(BTNodeState.SHUTDOWN)

    def _do_reset(self) -> Result[BTNodeState, BehaviorTreeException]:
        for child in self.children:
            return child.reset()
        return Ok(BTNodeState.IDLE)

    def _do_untick(self) -> Result[BTNodeState, BehaviorTreeException]:
        for child in self.children:
            return child.untick()
        return Ok(BTNodeState.IDLE)


@define_bt_node(NodeConfig(inputs={}, outputs={}, max_children=1))
class Inverter(Decorator):
    """
    Inverts the result of the child.

    Return SUCCEEDED if the child returned FAILED,
    return FAILED if the child returned SUCCEEDED.

    RUNNING is forwarded
    """

    def _do_setup(self) -> Result[BTNodeState, BehaviorTreeException]:
        for child in self.children:
            return child.setup()
        return Ok(BTNodeState.IDLE)

    def _do_tick(self) -> Result[BTNodeState, BehaviorTreeException]:
        for child in self.children:
            return child.tick().map(
                lambda state: (
                    BTNodeState.SUCCEEDED
                    if state == BTNodeState.FAILED
                    else BTNodeState.FAILED if state == BTNodeState.SUCCEEDED else state
                )
            )

        # Succeed if we have no children
        return Ok(BTNodeState.SUCCEEDED)

    def _do_shutdown(self) -> Result[BTNodeState, BehaviorTreeException]:
        return Ok(BTNodeState.SHUTDOWN)

    def _do_reset(self) -> Result[BTNodeState, BehaviorTreeException]:
        for child in self.children:
            return child.reset()
        return Ok(BTNodeState.IDLE)

    def _do_untick(self) -> Result[BTNodeState, BehaviorTreeException]:
        for child in self.children:
            return child.untick()
        return Ok(BTNodeState.IDLE)


@define_bt_node(
    NodeConfig(
        inputs={"num_retries": IntType(min_value=0, allow_dynamic=False)},
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

    def _do_setup(self) -> Result[BTNodeState, BehaviorTreeException]:
        self._retry_count = 0
        for child in self.children:
            return child.setup()
        match self.inputs.get_value_as("num_retries", int):
            case Err(e):
                return Err(e)
            case Ok(n):
                self._retry_limit = n
        return Ok(BTNodeState.IDLE)

    def _do_tick(self) -> Result[BTNodeState, BehaviorTreeException]:
        for child in self.children:
            match child.tick():
                case Err(e):
                    return Err(e)
                case Ok(s):
                    state = s

            if state == BTNodeState.FAILED:
                if self._retry_count < self._retry_limit:
                    self._retry_count += 1
                    return child.reset().map(lambda _: BTNodeState.RUNNING)
                else:
                    self._retry_count = 0
                    return Ok(BTNodeState.FAILED)
            return Ok(state)

        # Succeed if we have no children
        return Ok(BTNodeState.SUCCEEDED)

    def _do_shutdown(self) -> Result[BTNodeState, BehaviorTreeException]:
        return Ok(BTNodeState.SHUTDOWN)

    def _do_reset(self) -> Result[BTNodeState, BehaviorTreeException]:
        self._retry_count = 0
        for child in self.children:
            return child.reset()
        return Ok(BTNodeState.IDLE)

    def _do_untick(self) -> Result[BTNodeState, BehaviorTreeException]:
        for child in self.children:
            return child.untick()
        return Ok(BTNodeState.IDLE)


@define_bt_node(
    NodeConfig(
        inputs={"num_repeats": IntType(min_value=1, allow_dynamic=False)},
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

    def _do_setup(self) -> Result[BTNodeState, BehaviorTreeException]:
        self._repeat_count = 0
        match self.inputs.get_value_as("num_repeats", int):
            case Err(e):
                return Err(e)
            case Ok(n):
                self._num_repeats = n
        for child in self.children:
            return child.setup()
        return Ok(BTNodeState.IDLE)

    def _do_tick(self) -> Result[BTNodeState, BehaviorTreeException]:
        for child in self.children:
            match child.tick():
                case Err(e):
                    return Err(e)
                case Ok(s):
                    state = s
            if state == BTNodeState.SUCCEEDED:
                if self._repeat_count < self._num_repeats:
                    self._repeat_count += 1
                    return child.reset().map(lambda _: BTNodeState.RUNNING)
                else:
                    return Ok(BTNodeState.SUCCEEDED)
            return Ok(state)

        # Succeed if we have no children
        return Ok(BTNodeState.SUCCEEDED)

    def _do_shutdown(self) -> Result[BTNodeState, BehaviorTreeException]:
        return Ok(BTNodeState.SHUTDOWN)

    def _do_reset(self) -> Result[BTNodeState, BehaviorTreeException]:
        self._repeat_count = 0
        for child in self.children:
            return child.reset()
        return Ok(BTNodeState.IDLE)

    def _do_untick(self) -> Result[BTNodeState, BehaviorTreeException]:
        for child in self.children:
            return child.untick()
        return Ok(BTNodeState.IDLE)


@define_bt_node(
    NodeConfig(
        inputs={"reset": BoolType(allow_static=False)},
        outputs={},
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

    def _do_tick(self) -> Result[BTNodeState, BehaviorTreeException]:
        match self.inputs.get_value_as("reset", bool):
            case Err(e):
                return Err(e)
            case Ok(b):
                reset = b
        if reset:
            self._repeat_count = 0

        # Only TICK the children
        if self._repeat_count < self._num_repeats:
            for child in self.children:
                match child.tick():
                    case Err(e):
                        return Err(e)
                    case Ok(s):
                        state = s
                if state == BTNodeState.SUCCEEDED:
                    if self._repeat_count < self._num_repeats:
                        self._repeat_count += 1
                        return child.reset().map(lambda _: BTNodeState.RUNNING)
                    else:
                        return Ok(BTNodeState.SUCCEEDED)
                return Ok(state)

        # Succeed if we have no children
        return Ok(BTNodeState.SUCCEEDED)

    def _do_reset(self) -> Result[BTNodeState, BehaviorTreeException]:
        # Only reset childs if we havent reached our goal
        if self._repeat_count < self._num_repeats:
            for child in self.children:
                return child.reset()
        return Ok(BTNodeState.IDLE)


@define_bt_node(NodeConfig(inputs={}, outputs={}, max_children=1))
class RepeatAlways(Decorator):
    """
    Repeats the child an infinite number of times.

    returns RUNNING if there is a child, regardless of the childs returned state,

    returns SUCCEEDED if there is no child.

    """

    def _do_setup(self) -> Result[BTNodeState, BehaviorTreeException]:
        for child in self.children:
            return child.setup()
        return Ok(BTNodeState.IDLE)

    def _do_tick(self) -> Result[BTNodeState, BehaviorTreeException]:
        for child in self.children:
            match child.tick():
                case Err(e):
                    return Err(e)
                case Ok(s):
                    state = s
            if state != BTNodeState.RUNNING:
                match child.reset():
                    case Err(e):
                        return Err(e)
                    case Ok(_):
                        pass
            return Ok(BTNodeState.RUNNING)

        # Succeed if we have no children
        return Ok(BTNodeState.SUCCEEDED)

    def _do_shutdown(self) -> Result[BTNodeState, BehaviorTreeException]:
        return Ok(BTNodeState.SHUTDOWN)

    def _do_reset(self) -> Result[BTNodeState, BehaviorTreeException]:
        for child in self.children:
            return child.reset()
        return Ok(BTNodeState.IDLE)

    def _do_untick(self) -> Result[BTNodeState, BehaviorTreeException]:
        for child in self.children:
            return child.untick()
        return Ok(BTNodeState.IDLE)


@define_bt_node(NodeConfig(inputs={}, outputs={}, max_children=1))
class RepeatUntilFail(Decorator):
    """
    Repeats the child an infinite number of times until it returns FAILED.

    returns RUNNING if the child is RUNNING or has returned SUCCEEDED,
    returns FAILED if the child returned FAILED

    returns SUCCEEDED if there is no child.

    """

    def _do_setup(self) -> Result[BTNodeState, BehaviorTreeException]:
        for child in self.children:
            return child.setup()
        return Ok(BTNodeState.IDLE)

    def _do_tick(self) -> Result[BTNodeState, BehaviorTreeException]:
        for child in self.children:
            match child.tick():
                case Err(e):
                    return Err(e)
                case Ok(s):
                    state = s
            if state == BTNodeState.SUCCEEDED:
                return child.reset().map(lambda _: BTNodeState.RUNNING)
            return Ok(state)

        # Succeed if we have no children
        return Ok(BTNodeState.SUCCEEDED)

    def _do_shutdown(self) -> Result[BTNodeState, BehaviorTreeException]:
        return Ok(BTNodeState.SHUTDOWN)

    def _do_reset(self) -> Result[BTNodeState, BehaviorTreeException]:
        for child in self.children:
            return child.reset()
        return Ok(BTNodeState.IDLE)

    def _do_untick(self) -> Result[BTNodeState, BehaviorTreeException]:
        for child in self.children:
            return child.untick()
        return Ok(BTNodeState.IDLE)


@define_bt_node(NodeConfig(inputs={}, outputs={}, max_children=1))
class RepeatIfFail(Decorator):
    """
    Repeats the child an infinite number of times if it returns FAILED.

    returns RUNNING if the child is RUNNING,
    returns FAILED if the child returned FAILED

    returns SUCCEEDED if the child SUCCEEDED or if there is no child.

    """

    def _do_setup(self) -> Result[BTNodeState, BehaviorTreeException]:
        for child in self.children:
            return child.setup()
        return Ok(BTNodeState.IDLE)

    def _do_tick(self) -> Result[BTNodeState, BehaviorTreeException]:
        for child in self.children:
            match child.tick():
                case Err(e):
                    return Err(e)
                case Ok(s):
                    state = s
            if state == BTNodeState.FAILED:
                return child.reset().map(lambda _: BTNodeState.RUNNING)
            return Ok(state)

        # Succeed if we have no children
        return Ok(BTNodeState.SUCCEEDED)

    def _do_shutdown(self) -> Result[BTNodeState, BehaviorTreeException]:
        return Ok(BTNodeState.SHUTDOWN)

    def _do_reset(self) -> Result[BTNodeState, BehaviorTreeException]:
        for child in self.children:
            return child.reset()
        return Ok(BTNodeState.IDLE)

    def _do_untick(self) -> Result[BTNodeState, BehaviorTreeException]:
        for child in self.children:
            return child.untick()
        return Ok(BTNodeState.IDLE)


@define_bt_node(NodeConfig(inputs={}, outputs={}, max_children=1))
class Optional(Decorator):
    """
    Wraps a child that may not be able to execute.

    A child that cannot execute will not be ticked. Instead, this
    decorator will always return SUCCEEDED.

    If the child *can* execute, the decorator will simply forward all
    events.

    """

    def _do_setup(self) -> Result[BTNodeState, BehaviorTreeException]:
        self.execute_child = False
        for child in self.children:
            match child.calculate_utility():
                case Err(e):
                    return Err(e)
                case Ok(u):
                    utility_bounds = u
            if utility_bounds.can_execute:
                self.execute_child = True
                return child.setup()
        return Ok(BTNodeState.IDLE)

    def _do_tick(self) -> Result[BTNodeState, BehaviorTreeException]:
        if self.execute_child:
            return self.children[0].tick()
        else:
            return Ok(BTNodeState.SUCCEEDED)

    def _do_shutdown(self) -> Result[BTNodeState, BehaviorTreeException]:
        return Ok(BTNodeState.SHUTDOWN)

    def _do_reset(self) -> Result[BTNodeState, BehaviorTreeException]:
        if self.execute_child:
            return self.children[0].reset()
        else:
            return Ok(BTNodeState.IDLE)

    def _do_untick(self) -> Result[BTNodeState, BehaviorTreeException]:
        if self.execute_child:
            return self.children[0].untick()
        else:
            return Ok(BTNodeState.IDLE)

    def _do_calculate_utility(self) -> Result[UtilityBounds, BehaviorTreeException]:
        for child in self.children:
            match child.calculate_utility():
                case Err(e):
                    return Err(e)
                case Ok(u):
                    utility_bounds = u
            if utility_bounds.can_execute:
                return Ok(utility_bounds)

        # If the child can't execute, return a UtilityBounds object
        # that can execute, but does not have any bounds set (that is,
        # if another executor can actually execute our child, it is
        # pretty much guaranteed to have a better Utility score than
        # us)
        return Ok(UtilityBounds(can_execute=True))


@define_bt_node(
    NodeConfig(
        inputs={"watch": StringType(allow_static=False)},
        outputs={},
        max_children=1,
    )
)
class Watch(Decorator):
    """Untick child if watch string changed."""

    def _do_setup(self) -> Result[BTNodeState, BehaviorTreeException]:
        for child in self.children:
            return child.setup()
        return Ok(BTNodeState.IDLE)

    def _do_tick(self) -> Result[BTNodeState, BehaviorTreeException]:
        if len(self.children) == 0:
            return Ok(BTNodeState.SUCCEEDED)

        match self.inputs.any_updated("watch"):
            case Err(e):
                return Err(e)
            case Ok(b):
                updated = b

        child = self.children[0]
        if updated:
            match child.untick():
                case Err(e):
                    return Err(e)
                case Ok(_):
                    pass

        return self.children[0].tick()

    def _do_shutdown(self) -> Result[BTNodeState, BehaviorTreeException]:
        return Ok(BTNodeState.SHUTDOWN)

    def _do_reset(self) -> Result[BTNodeState, BehaviorTreeException]:
        for child in self.children:
            return child.reset()
        return Ok(BTNodeState.IDLE)

    def _do_untick(self) -> Result[BTNodeState, BehaviorTreeException]:
        for child in self.children:
            return child.untick()
        return Ok(BTNodeState.IDLE)
