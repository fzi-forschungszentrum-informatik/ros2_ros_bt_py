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
from time import time

from ros_bt_py.vendor.result import Result, Ok, Err

from ros_bt_py.node import Leaf, define_bt_node
from ros_bt_py.node_config import NodeConfig
from ros_bt_py.helpers import BTNodeState
from ros_bt_py.exceptions import BehaviorTreeException


@define_bt_node(
    NodeConfig(
        version="0.1.0",
        options={"seconds_to_wait": float},
        inputs={},
        outputs={},
        max_children=0,
    )
)
class Wait(Leaf):
    """
    Returns "RUNNING" until at least the specified amount of seconds are elapsed.

    This is naturally not extremely precise because it depends on the tick interval

    If `seconds_to_wait` is 0 or negative, the node will immediately succeed
    """

    def _do_setup(self) -> Result[BTNodeState, BehaviorTreeException]:
        self.first_tick = True
        return Ok(BTNodeState.IDLE)

    def _do_tick(self) -> Result[BTNodeState, BehaviorTreeException]:
        now = time()
        if self.first_tick:
            self.start_time = now
            self.end_time = self.start_time + self.options["seconds_to_wait"]
            self.first_tick = False
        if now >= self.end_time:
            return Ok(BTNodeState.SUCCEEDED)
        else:
            return Ok(BTNodeState.RUNNING)

    def _do_shutdown(self) -> Result[BTNodeState, BehaviorTreeException]:
        self.first_tick = True
        return Ok(BTNodeState.SHUTDOWN)

    def _do_reset(self) -> Result[BTNodeState, BehaviorTreeException]:
        self.first_tick = True
        return Ok(BTNodeState.IDLE)

    def _do_untick(self) -> Result[BTNodeState, BehaviorTreeException]:
        return Ok(BTNodeState.IDLE)


@define_bt_node(
    NodeConfig(
        version="0.1.0",
        options={},
        inputs={"seconds_to_wait": float},
        outputs={},
        max_children=0,
    )
)
class WaitInput(Leaf):
    """
    Returns "RUNNING" until at least the specified amount of seconds are elapsed.

    This is naturally not extremely precise because it depends on the tick interval

    If `seconds_to_wait` is 0 or negative, the node will immediately succeed
    """

    def _do_setup(self) -> Result[BTNodeState, BehaviorTreeException]:
        self.first_tick = True
        return Ok(BTNodeState.IDLE)

    def _do_tick(self) -> Result[BTNodeState, BehaviorTreeException]:
        now = time()
        if self.first_tick:
            self.start_time = now
            self.end_time = self.start_time + self.inputs["seconds_to_wait"]
            self.first_tick = False
        if now >= self.end_time:
            return Ok(BTNodeState.SUCCEEDED)
        else:
            return Ok(BTNodeState.RUNNING)

    def _do_shutdown(self) -> Result[BTNodeState, BehaviorTreeException]:
        self.first_tick = True
        return Ok(BTNodeState.SHUTDOWN)

    def _do_reset(self) -> Result[BTNodeState, BehaviorTreeException]:
        self.first_tick = True
        return Ok(BTNodeState.IDLE)

    def _do_untick(self) -> Result[BTNodeState, BehaviorTreeException]:
        return Ok(BTNodeState.IDLE)
