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

from ros_bt_py.vendor.result import Result, Ok, Err

from ros_bt_py.helpers import BTNodeState

from ros_bt_py_interfaces.srv import ControlTreeExecution

from tests.integration.conftest import (
    TreeControlNode,
    TimeControlNode,
    sim_time_tree_node,
)

from tests.integration.flow_control_isolation.conftest import (
    MultiPublishNode,
    verify_node_states,
)


@pytest.mark.launch(fixture=sim_time_tree_node)
@pytest.mark.dependency(depends=["TopicMemorySubscriber"], scope="session")
def test_tree_load(
    tree_control_node: TreeControlNode,
    time_control_node: TimeControlNode,
):
    time_control_node.set_time(10)

    load_result = tree_control_node.load_tree(
        "trees/flow_control_isolation/sequence.yaml"
    )
    assert load_result.is_ok()


@pytest.mark.launch(fixture=sim_time_tree_node)
@pytest.mark.dependency(depends=["test_tree_load"])
def test_first_tick_running(
    tree_control_node: TreeControlNode,
    time_control_node: TimeControlNode,
):
    test_tree_load(tree_control_node, time_control_node)

    run_result = tree_control_node.execute_tree(ControlTreeExecution.Request.TICK_ONCE)
    assert run_result.is_ok()

    expected_states = {
        "Sequence": BTNodeState.RUNNING,
        "Child1": BTNodeState.RUNNING,
        "Child2": BTNodeState.IDLE,
        "Child3": BTNodeState.IDLE,
    }

    verify_node_states(tree_control_node, expected_states)


@pytest.mark.launch(fixture=sim_time_tree_node)
@pytest.mark.dependency(depends=["test_first_tick_running"])
def test_second_tick_running(
    tree_control_node: TreeControlNode,
    time_control_node: TimeControlNode,
    multi_publish_node: MultiPublishNode,
):
    test_first_tick_running(tree_control_node, time_control_node)

    multi_publish_node.publish_topic_1()

    # Give the tree time to process callbacks
    time.sleep(0.1)

    run_result = tree_control_node.execute_tree(ControlTreeExecution.Request.TICK_ONCE)
    assert run_result.is_ok()

    expected_states = {
        "Sequence": BTNodeState.RUNNING,
        "Child1": BTNodeState.SUCCEEDED,
        "Child2": BTNodeState.RUNNING,
        "Child3": BTNodeState.IDLE,
    }

    verify_node_states(tree_control_node, expected_states)


@pytest.mark.launch(fixture=sim_time_tree_node)
@pytest.mark.dependency(depends=["test_second_tick_running"])
def test_third_tick_running(
    tree_control_node: TreeControlNode,
    time_control_node: TimeControlNode,
    multi_publish_node: MultiPublishNode,
):
    test_second_tick_running(
        tree_control_node,
        time_control_node,
        multi_publish_node,
    )

    multi_publish_node.publish_topic_2()

    # Give the tree time to process callbacks
    time.sleep(0.1)

    run_result = tree_control_node.execute_tree(ControlTreeExecution.Request.TICK_ONCE)
    assert run_result.is_ok()

    expected_states = {
        "Sequence": BTNodeState.RUNNING,
        "Child1": BTNodeState.SUCCEEDED,
        "Child2": BTNodeState.SUCCEEDED,
        "Child3": BTNodeState.RUNNING,
    }

    verify_node_states(tree_control_node, expected_states)


@pytest.mark.launch(fixture=sim_time_tree_node)
@pytest.mark.dependency(depends=["test_third_tick_running"])
def test_forth_tick_success(
    tree_control_node: TreeControlNode,
    time_control_node: TimeControlNode,
    multi_publish_node: MultiPublishNode,
):
    test_third_tick_running(
        tree_control_node,
        time_control_node,
        multi_publish_node,
    )

    multi_publish_node.publish_topic_3()

    # Give the tree time to process callbacks
    time.sleep(0.1)

    run_result = tree_control_node.execute_tree(ControlTreeExecution.Request.TICK_ONCE)
    assert run_result.is_ok()

    expected_states = {
        "Sequence": BTNodeState.SUCCEEDED,
        "Child1": BTNodeState.SUCCEEDED,
        "Child2": BTNodeState.SUCCEEDED,
        "Child3": BTNodeState.SUCCEEDED,
    }

    verify_node_states(tree_control_node, expected_states)


@pytest.mark.launch(fixture=sim_time_tree_node)
@pytest.mark.dependency(depends=["test_forth_tick_success"])
def test_auto_reset_after_success(
    tree_control_node: TreeControlNode,
    time_control_node: TimeControlNode,
    multi_publish_node: MultiPublishNode,
):
    test_forth_tick_success(
        tree_control_node,
        time_control_node,
        multi_publish_node,
    )

    run_result = tree_control_node.execute_tree(ControlTreeExecution.Request.TICK_ONCE)
    assert run_result.is_ok()

    expected_states = {
        "Sequence": BTNodeState.RUNNING,
        "Child1": BTNodeState.RUNNING,
        "Child2": BTNodeState.IDLE,
        "Child3": BTNodeState.IDLE,
    }

    verify_node_states(tree_control_node, expected_states)


@pytest.mark.launch(fixture=sim_time_tree_node)
@pytest.mark.dependency(depends=["test_second_tick_running"])
def test_timeout_first_topic(
    tree_control_node: TreeControlNode,
    time_control_node: TimeControlNode,
    multi_publish_node: MultiPublishNode,
):
    test_second_tick_running(
        tree_control_node,
        time_control_node,
        multi_publish_node,
    )

    # Trigger timeout for Child1
    time_control_node.set_time(15)

    time.sleep(0.1)

    run_result = tree_control_node.execute_tree(ControlTreeExecution.Request.TICK_ONCE)
    assert run_result.is_ok()

    expected_states = {
        "Sequence": BTNodeState.FAILED,
        "Child1": BTNodeState.FAILED,
        "Child2": BTNodeState.IDLE,
        "Child3": BTNodeState.IDLE,
    }

    verify_node_states(tree_control_node, expected_states)


@pytest.mark.launch(fixture=sim_time_tree_node)
@pytest.mark.dependency(depends=["test_timeout_first_topic"])
def test_auto_reset_after_failure(
    tree_control_node: TreeControlNode,
    time_control_node: TimeControlNode,
    multi_publish_node: MultiPublishNode,
):
    test_timeout_first_topic(
        tree_control_node,
        time_control_node,
        multi_publish_node,
    )

    run_result = tree_control_node.execute_tree(ControlTreeExecution.Request.TICK_ONCE)
    assert run_result.is_ok()

    expected_states = {
        "Sequence": BTNodeState.RUNNING,
        "Child1": BTNodeState.RUNNING,
        "Child2": BTNodeState.IDLE,
        "Child3": BTNodeState.IDLE,
    }

    verify_node_states(tree_control_node, expected_states)


# This marker name can be used for other tests to depend on,
#   in case they rely on this node to work properly.
# NOTE The dependencies for this test should be set in a way
#   that includes all tests in the module.
@pytest.mark.dependency(
    name="Sequence",
    depends=[
        "test_auto_reset_after_success",
        "test_auto_reset_after_failure",
    ],
)
def test_confirm_node_function():
    # This test is purely for dependency handling and doesn't do anything on its own.
    pass
