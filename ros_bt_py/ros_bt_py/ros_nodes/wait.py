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

from ros_bt_py.data_types import FloatType
from ros_bt_py.exceptions import BehaviorTreeException
from ros_bt_py.helpers import BTNodeState
from ros_bt_py.node import Leaf, define_bt_node
from ros_bt_py.node_config import NodeConfig


@define_bt_node(
    NodeConfig(
        inputs={"seconds_to_wait": FloatType(allow_dynamic=False, min_value=0)},
        outputs={},
        max_children=0,
    )
)
class Wait(Leaf):
    """
    Returns "RUNNING" until at least the specified amount of seconds are elapsed.

    This is naturally not extremely precise because it depends on the tick interval

    If `seconds_to_wait` is 0, the node will immediately succeed
    """

    def _do_setup(self) -> Result[BTNodeState, BehaviorTreeException]:
        self.start_time = None
        if not self.has_ros_node:
            error_msg = f"{self.name} does not have a reference to a ROS node"
            self.logerr(error_msg)
            return Err(BehaviorTreeException(error_msg))
        match self.inputs.get_value_as("seconds_to_wait", float):
            case Err(e):
                return Err(e)
            case Ok(t):
                self.wait_duration = t
        return Ok(BTNodeState.IDLE)

    def _do_tick(self) -> Result[BTNodeState, BehaviorTreeException]:
        now = self.ros_node.get_clock().now()
        if self.start_time is None:
            self.start_time = now
        seconds_since_call: float = (
            self.ros_node.get_clock().now() - self.start_time
        ).nanoseconds / 1e9
        if seconds_since_call >= self.wait_duration:
            return Ok(BTNodeState.SUCCEEDED)
        else:
            return Ok(BTNodeState.RUNNING)

    def _do_shutdown(self) -> Result[BTNodeState, BehaviorTreeException]:
        return Ok(BTNodeState.SHUTDOWN)

    def _do_reset(self) -> Result[BTNodeState, BehaviorTreeException]:
        self.start_time = None
        return Ok(BTNodeState.IDLE)

    def _do_untick(self) -> Result[BTNodeState, BehaviorTreeException]:
        return Ok(BTNodeState.IDLE)
