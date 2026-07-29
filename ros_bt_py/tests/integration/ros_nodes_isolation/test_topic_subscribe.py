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

from rclpy.publisher import Publisher
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)

from ros_bt_py.vendor.result import Ok, Err

from example_interfaces.msg import Empty

from ros_bt_py_interfaces.msg import NodeState
from ros_bt_py_interfaces.srv import ControlTreeExecution

from tests.integration.conftest import TreeControlNode, standard_tree_node


@pytest.fixture
def foo_publisher(tree_control_node: TreeControlNode):
    msg_publisher = tree_control_node.create_publisher(
        Empty,
        "/foo",
        QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        ),
    )
    return msg_publisher


@pytest.mark.launch(fixture=standard_tree_node)
@pytest.mark.dependency()
def test_tree_load(tree_control_node: TreeControlNode):
    load_result = tree_control_node.load_tree(
        "trees/ros_nodes_isolation/topic_subscribe.yaml"
    )
    assert load_result.is_ok()


@pytest.mark.launch(fixture=standard_tree_node)
@pytest.mark.dependency(depends=["test_tree_load"])
def test_first_tick_running(tree_control_node: TreeControlNode):
    test_tree_load(tree_control_node)

    run_result = tree_control_node.execute_tree(ControlTreeExecution.Request.TICK_ONCE)
    assert run_result.is_ok()

    match tree_control_node.get_tree_state():
        case Err(e):
            assert False, e
        case Ok(s):
            state: NodeState = s.node_states[0]  # type: ignore
    assert state.state == NodeState.RUNNING


@pytest.mark.launch(fixture=standard_tree_node)
@pytest.mark.dependency(depends=["test_first_tick_running"])
def test_receive_msg(
    tree_control_node: TreeControlNode,
    foo_publisher: Publisher,
):
    test_first_tick_running(tree_control_node)

    foo_publisher.publish(Empty())

    # Give the tree time to process callbacks (simulate tick frequency)
    time.sleep(0.1)

    # We expect the node to switch to SUCCEEDED after receiving the message
    run_result = tree_control_node.execute_tree(ControlTreeExecution.Request.TICK_ONCE)
    assert run_result.is_ok()

    match tree_control_node.get_tree_state():
        case Err(e):
            assert False, e
        case Ok(s):
            state: NodeState = s.node_states[0]  # type: ignore
    assert state.state == NodeState.SUCCEEDED


@pytest.mark.launch(fixture=standard_tree_node)
@pytest.mark.dependency(depends=["test_receive_msg"])
def test_running_after_msg(
    tree_control_node: TreeControlNode,
    foo_publisher: Publisher,
):
    test_receive_msg(tree_control_node, foo_publisher)

    # Another tick should set it back to RUNNING
    run_result = tree_control_node.execute_tree(ControlTreeExecution.Request.TICK_ONCE)
    assert run_result.is_ok()

    match tree_control_node.get_tree_state():
        case Err(e):
            assert False, e
        case Ok(s):
            state: NodeState = s.node_states[0]  # type: ignore
    assert state.state == NodeState.RUNNING


@pytest.mark.launch(fixture=standard_tree_node)
@pytest.mark.dependency(depends=["test_running_after_msg"])
def test_receive_after_reset(
    tree_control_node: TreeControlNode,
    foo_publisher: Publisher,
):
    test_running_after_msg(tree_control_node, foo_publisher)

    # Since the topic we defined is latching, it should eventually succeed after a reset.
    run_result = tree_control_node.execute_tree(ControlTreeExecution.Request.RESET)
    assert run_result.is_ok()

    # First tick to recreate the subscriber
    run_result = tree_control_node.execute_tree(ControlTreeExecution.Request.TICK_ONCE)
    assert run_result.is_ok()

    # Give the tree time to process callbacks (simulate tick frequency)
    time.sleep(0.1)

    # Second tick receives the latched message
    run_result = tree_control_node.execute_tree(ControlTreeExecution.Request.TICK_ONCE)
    assert run_result.is_ok()

    match tree_control_node.get_tree_state():
        case Err(e):
            assert False, e
        case Ok(s):
            state: NodeState = s.node_states[0]  # type: ignore
    assert state.state == NodeState.SUCCEEDED


@pytest.mark.launch(fixture=standard_tree_node)
@pytest.mark.dependency(depends=["test_receive_after_reset"])
def test_running_after_reset_msg(
    tree_control_node: TreeControlNode,
    foo_publisher: Publisher,
):
    test_receive_after_reset(tree_control_node, foo_publisher)

    # And then go back to RUNNING after another tick
    run_result = tree_control_node.execute_tree(ControlTreeExecution.Request.TICK_ONCE)
    assert run_result.is_ok()

    match tree_control_node.get_tree_state():
        case Err(e):
            assert False, e
        case Ok(s):
            state: NodeState = s.node_states[0]  # type: ignore
    assert state.state == NodeState.RUNNING


# This marker name can be used for other tests to depend on,
#   in case they rely on this node to work properly.
# NOTE The dependencies for this test should be set in a way
#   that includes all tests in the module.
@pytest.mark.dependency(
    name="TopicSubscriber",
    depends=[
        "test_running_after_reset_msg",
    ],
)
def test_confirm_node_function():
    # This test is purely for dependency handling and doesn't do anything on its own.
    pass
