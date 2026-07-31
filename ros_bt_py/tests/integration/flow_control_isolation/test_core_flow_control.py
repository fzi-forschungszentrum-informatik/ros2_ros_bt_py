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
from tests.integration.conftest import TreeControlNode, standard_tree_node
from tests.integration.flow_control_isolation.conftest import verify_node_states

from ros_bt_py_interfaces.srv import ControlTreeExecution


def load_and_tick(tree_control_node: TreeControlNode, tree_name: str) -> None:
    load_result = tree_control_node.load_tree(
        f"trees/flow_control_isolation/{tree_name}.yaml"
    )
    assert load_result.is_ok(), load_result
    tick(tree_control_node)


def tick(tree_control_node: TreeControlNode) -> None:
    run_result = tree_control_node.execute_tree(ControlTreeExecution.Request.TICK_ONCE)
    assert run_result.is_ok(), run_result


@pytest.mark.launch(fixture=standard_tree_node)
def test_name_switch_fails_without_a_matching_child(tree_control_node: TreeControlNode):
    load_and_tick(tree_control_node, "name_switch")

    verify_node_states(
        tree_control_node,
        {
            "NameSwitch": BTNodeState.FAILED,
            "Selected": BTNodeState.IDLE,
            "Unselected": BTNodeState.IDLE,
        },
    )


@pytest.mark.launch(fixture=standard_tree_node)
def test_fallback_handles_empty_and_failed_children_before_success(
    tree_control_node: TreeControlNode,
):
    load_and_tick(tree_control_node, "fallback")

    verify_node_states(
        tree_control_node,
        {
            "Fallback": BTNodeState.SUCCEEDED,
            "EmptyFallback": BTNodeState.FAILED,
            "AllFailedFallback": BTNodeState.FAILED,
            "Success": BTNodeState.SUCCEEDED,
            "UntickedRunning": BTNodeState.IDLE,
        },
    )


@pytest.mark.launch(fixture=standard_tree_node)
def test_memory_fallback_keeps_running_child(tree_control_node: TreeControlNode):
    load_and_tick(tree_control_node, "memory_fallback")
    tick(tree_control_node)

    verify_node_states(
        tree_control_node,
        {
            "MemoryFallback": BTNodeState.RUNNING,
            "FailedBeforeRunning": BTNodeState.FAILED,
            "RememberedRunning": BTNodeState.RUNNING,
            "UntickedSuccess": BTNodeState.IDLE,
        },
    )


@pytest.mark.launch(fixture=standard_tree_node)
def test_parallel_reaches_threshold_and_auto_restarts(
    tree_control_node: TreeControlNode,
):
    load_and_tick(tree_control_node, "parallel")
    verify_node_states(
        tree_control_node,
        {
            "Parallel": BTNodeState.SUCCEEDED,
            "SuccessOne": BTNodeState.SUCCEEDED,
            "SuccessTwo": BTNodeState.SUCCEEDED,
            "UntickedRunning": BTNodeState.IDLE,
        },
    )

    tick(tree_control_node)
    verify_node_states(tree_control_node, {"Parallel": BTNodeState.SUCCEEDED})


@pytest.mark.launch(fixture=standard_tree_node)
def test_parallel_failure_tolerance_keeps_running_within_tolerance(
    tree_control_node: TreeControlNode,
):
    load_and_tick(tree_control_node, "parallel_failure_tolerance")

    verify_node_states(
        tree_control_node,
        {
            "ParallelFailureTolerance": BTNodeState.RUNNING,
            "ToleratedFailure": BTNodeState.FAILED,
            "Success": BTNodeState.SUCCEEDED,
            "Running": BTNodeState.RUNNING,
        },
    )
