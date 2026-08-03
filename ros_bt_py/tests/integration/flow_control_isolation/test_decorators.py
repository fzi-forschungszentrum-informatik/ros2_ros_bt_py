# Copyright (c) 2026 FZI Forschungszentrum Informatik
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
#    * Neither the name of the copyright holder nor the names of its
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

import pytest

import time

from ros_bt_py.helpers import BTNodeState
from ros_bt_py_interfaces.srv import ControlTreeExecution
from tests.integration.conftest import TreeControlNode, standard_tree_node
from tests.integration.flow_control_isolation.conftest import verify_node_states
from tests.integration.flow_control_isolation.conftest import MultiPublishNode


@pytest.mark.launch(fixture=standard_tree_node)
def test_decorator_state_conversion_and_counters(tree_control_node: TreeControlNode):
    assert tree_control_node.load_tree(
        "trees/flow_control_isolation/decorators.yaml"
    ).is_ok()

    for _ in range(3):
        assert tree_control_node.execute_tree(
            ControlTreeExecution.Request.TICK_ONCE
        ).is_ok()

    verify_node_states(
        tree_control_node,
        {
            "IgnoreFailure": BTNodeState.SUCCEEDED,
            "IgnoreSuccess": BTNodeState.FAILED,
            "UntilSuccess": BTNodeState.SUCCEEDED,
            "Inverter": BTNodeState.SUCCEEDED,
            "Repeat": BTNodeState.SUCCEEDED,
            "RepeatAlways": BTNodeState.RUNNING,
            "RepeatUntilFail": BTNodeState.FAILED,
            "RepeatIfFail": BTNodeState.SUCCEEDED,
            "OptionalExecutable": BTNodeState.SUCCEEDED,
        },
    )


@pytest.mark.launch(fixture=standard_tree_node)
def test_live_decorators(
    tree_control_node: TreeControlNode, multi_publish_node: MultiPublishNode
):
    assert tree_control_node.load_tree(
        "trees/flow_control_isolation/decorators_live.yaml"
    ).is_ok()
    assert tree_control_node.execute_tree(
        ControlTreeExecution.Request.TICK_ONCE
    ).is_ok()
    verify_node_states(
        tree_control_node,
        {
            "IgnoreRunning": BTNodeState.SUCCEEDED,
            "OptionalNotExecutable": BTNodeState.SUCCEEDED,
            "Watch": BTNodeState.RUNNING,
        },
    )

    multi_publish_node.publish_watch_topic()
    time.sleep(0.1)
    assert tree_control_node.execute_tree(
        ControlTreeExecution.Request.TICK_ONCE
    ).is_ok()
    verify_node_states(tree_control_node, {"Watch": BTNodeState.SUCCEEDED})
