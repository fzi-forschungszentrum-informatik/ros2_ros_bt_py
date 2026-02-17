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

from ros_bt_py.exceptions import BehaviorTreeException
from ros_bt_py.helpers import BTNodeState
from ros_bt_py.node import Decorator, define_bt_node
from ros_bt_py.node_config import NodeConfig


@define_bt_node(
    NodeConfig(
        version="0.1.0",
        options={"tick_interval": float},
        inputs={},
        outputs={},
        max_children=1,
    )
)
class Throttle(Decorator):
    """
    Wrap a child that stores its success and failures for tick_interval seconds.

    A child that SUCCEEDED or FAILED less than tick_interval seconds will not be ticked.
    This decorator will return the last result until tick_interval seconds elapsed.

    """

    def _do_setup(self) -> Result[BTNodeState, BehaviorTreeException]:
        if not self.has_ros_node:
            error_msg = f"{self.name} does not have a reference to a ROS node"
            self.logerr(error_msg)
            return Err(BehaviorTreeException(error_msg))

        self._last_tick = None
        self._last_result = Ok(BTNodeState.FAILED)
        for child in self.children:
            setup_result = child.setup()
            # TODO This seems unnecessary, since len(self.children) == 1 always
            if setup_result.is_err():
                return setup_result
        return setup_result

    def _do_tick(self) -> Result[BTNodeState, BehaviorTreeException]:
        current_time = self.ros_node.get_clock().now()
        if (
            self._last_tick is None
            or (current_time - self._last_tick).nanoseconds / 1e9
            > self.options["tick_interval"]
        ):
            for child in self.children:
                result = child.tick()
                if result.ok() == BTNodeState.RUNNING:
                    return result
                self._last_result = result
                self._last_tick = current_time
                # TODO Why do we reset on every result?
                reset_result = child.reset()
                if reset_result.is_err():
                    return reset_result
        return self._last_result

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
        version="0.1.0",
        options={"tick_interval": float},
        inputs={},
        outputs={},
        max_children=1,
    )
)
class ThrottleSuccess(Decorator):
    """
    Wrap a child that is prevented to SUCCEEDED multiple times in tick_interval seconds.

    A child that SUCCEEDED less than tick_interval seconds will not be ticked.
    This decorator will return SUCCEEDED once then FAILED until tick_interval seconds elapsed.

    """

    def _do_setup(self) -> Result[BTNodeState, BehaviorTreeException]:
        if not self.has_ros_node:
            error_msg = f"{self.name} does not have a reference to a ROS node"
            self.logerr(error_msg)
            return Err(BehaviorTreeException(error_msg))

        self._last_success_tick = None
        for child in self.children:
            setup_result = child.setup()
            # TODO This seems unnecessary, since len(self.children) == 1 always
            if setup_result.is_err():
                return setup_result
        return setup_result

    def _do_tick(self) -> Result[BTNodeState, BehaviorTreeException]:
        current_time = self.ros_node.get_clock().now()
        if (
            self._last_success_tick is None
            or (current_time - self._last_success_tick).nanoseconds / 1e9
            > self.options["tick_interval"]
        ):
            for child in self.children:
                result = child.tick()
                if result == BTNodeState.RUNNING:
                    return result
                if result == BTNodeState.SUCCEEDED:
                    self._last_success_tick = current_time
                    return result
        return Ok(BTNodeState.FAILED)

    def _do_shutdown(self) -> Result[BTNodeState, BehaviorTreeException]:
        self._last_success_tick = None
        return Ok(BTNodeState.SHUTDOWN)

    def _do_reset(self) -> Result[BTNodeState, BehaviorTreeException]:
        for child in self.children:
            return child.reset()
        return Ok(BTNodeState.IDLE)

    def _do_untick(self) -> Result[BTNodeState, BehaviorTreeException]:
        for child in self.children:
            return child.untick()
        return Ok(BTNodeState.IDLE)
