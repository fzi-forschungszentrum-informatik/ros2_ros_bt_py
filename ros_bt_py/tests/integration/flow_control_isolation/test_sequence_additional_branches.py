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

from ros_bt_py.helpers import BTNodeState
from ros_bt_py_interfaces.srv import ControlTreeExecution

from tests.integration.conftest import TreeControlNode, standard_tree_node
from tests.integration.flow_control_isolation.conftest import verify_node_states


def load_tree(tree_control_node: TreeControlNode, tree_name: str) -> None:
    result = tree_control_node.load_tree(
        f"trees/flow_control_isolation/{tree_name}.yaml"
    )
    assert result.is_ok(), result


def execute(tree_control_node: TreeControlNode, command: int) -> None:
    result = tree_control_node.execute_tree(command)
    assert result.is_ok(), result


@pytest.mark.launch(fixture=standard_tree_node)
@pytest.mark.parametrize(
    ("tree_name", "root_name"),
    [
        ("sequence_empty", "EmptySequence"),
        ("memory_sequence_empty", "EmptyMemorySequence"),
    ],
)
def test_empty_sequence_succeeds(
    tree_control_node: TreeControlNode, tree_name: str, root_name: str
):
    load_tree(tree_control_node, tree_name)
    execute(tree_control_node, ControlTreeExecution.Request.TICK_ONCE)

    verify_node_states(tree_control_node, {root_name: BTNodeState.SUCCEEDED})


@pytest.mark.launch(fixture=standard_tree_node)
def test_sequence_explicit_untick_and_reset(
    tree_control_node: TreeControlNode,
):
    root_name = "SequenceLifecycle"
    load_tree(tree_control_node, "sequence_lifecycle")
    execute(tree_control_node, ControlTreeExecution.Request.TICK_ONCE)
    verify_node_states(
        tree_control_node,
        {root_name: BTNodeState.RUNNING, "RunningChild": BTNodeState.RUNNING},
    )

    # STOP from WAITING_FOR_TICK calls untick on the actual tree root.
    execute(tree_control_node, ControlTreeExecution.Request.STOP)
    verify_node_states(
        tree_control_node,
        {root_name: BTNodeState.IDLE, "RunningChild": BTNodeState.PAUSED},
    )

    execute(tree_control_node, ControlTreeExecution.Request.TICK_ONCE)
    execute(tree_control_node, ControlTreeExecution.Request.RESET)
    verify_node_states(
        tree_control_node,
        {root_name: BTNodeState.IDLE, "RunningChild": BTNodeState.IDLE},
    )


@pytest.mark.launch(fixture=standard_tree_node)
def test_memory_sequence_returns_terminal_failure(tree_control_node: TreeControlNode):
    load_tree(tree_control_node, "memory_sequence_terminal_failure")
    execute(tree_control_node, ControlTreeExecution.Request.TICK_ONCE)

    verify_node_states(
        tree_control_node,
        {
            "MemorySequenceTerminalFailure": BTNodeState.FAILED,
            "Failure": BTNodeState.FAILED,
            "UntickedRunning": BTNodeState.PAUSED,
        },
    )
