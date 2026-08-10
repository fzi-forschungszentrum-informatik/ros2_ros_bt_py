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

from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from threading import Condition, Thread
import time

from ros_bt_py.vendor.result import Ok, Err

from example_interfaces.srv import Trigger

from ros_bt_py_interfaces.msg import NodeState
from ros_bt_py_interfaces.srv import ControlTreeExecution

from tests.integration.conftest import (
    TreeControlNode,
    TimeControlNode,
    sim_time_tree_node,
)


class FooService(Node):
    def __init__(self, *args, **kwargs):
        super().__init__("foo_service", *args, **kwargs)

        self._condition = Condition()
        self._has_request = False
        self._service_block = False

        _ = self.create_service(
            Trigger,
            "/foo",
            self._callback,
        )

    def _callback(self, request: Trigger.Request, response: Trigger.Response):
        with self._condition:
            self._has_request = True
            while self._service_block:
                self._condition.wait()
        response.success = True
        return response

    @property
    def has_request(self):
        with self._condition:
            return self._has_request

    def reset_request(self):
        with self._condition:
            self._has_request = False

    def set_service_block(self, block: bool):
        with self._condition:
            self._service_block = block
            self._condition.notify()


@pytest.fixture()
def foo_service():
    node = FooService()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = Thread(target=executor.spin)
    spin_thread.start()
    yield node
    # Allow bloacked callbacks to complete
    node.set_service_block(False)
    executor.shutdown(timeout_sec=30)
    spin_thread.join(timeout=30)
    node.destroy_node()


@pytest.mark.launch(fixture=sim_time_tree_node)
@pytest.mark.dependency()
def test_tree_load(
    tree_control_node: TreeControlNode,
    time_control_node: TimeControlNode,
):
    time_control_node.set_time(10)

    load_result = tree_control_node.load_tree("trees/ros_nodes_isolation/service.yaml")
    assert load_result.is_ok()


@pytest.mark.launch(fixture=sim_time_tree_node)
def test_shutdown_before_first_tick(
    tree_control_node: TreeControlNode,
    time_control_node: TimeControlNode,
):
    assert tree_control_node.execute_tree(ControlTreeExecution.Request.SHUTDOWN).is_ok()
    test_tree_load(tree_control_node, time_control_node)

    shutdown_result = tree_control_node.execute_tree(
        ControlTreeExecution.Request.SHUTDOWN
    )
    assert shutdown_result.is_ok()


@pytest.mark.launch(fixture=sim_time_tree_node)
@pytest.mark.dependency(depends=["test_tree_load"])
def test_send_request(
    tree_control_node: TreeControlNode,
    time_control_node: TimeControlNode,
    foo_service: FooService,
):
    test_tree_load(tree_control_node, time_control_node)

    foo_service.set_service_block(True)

    run_result = tree_control_node.execute_tree(ControlTreeExecution.Request.TICK_ONCE)
    assert run_result.is_ok()

    match tree_control_node.get_tree_state():
        case Err(e):
            assert False, e
        case Ok(s):
            state: NodeState = s.node_states[0]  # type: ignore
    assert state.state == NodeState.RUNNING

    time.sleep(0.1)

    assert foo_service.has_request


@pytest.mark.launch(fixture=sim_time_tree_node)
@pytest.mark.dependency(depends=["test_send_request"])
def test_succeed_response(
    tree_control_node: TreeControlNode,
    time_control_node: TimeControlNode,
    foo_service: FooService,
):
    test_send_request(
        tree_control_node,
        time_control_node,
        foo_service,
    )

    run_result = tree_control_node.execute_tree(ControlTreeExecution.Request.TICK_ONCE)
    assert run_result.is_ok()

    match tree_control_node.get_tree_state():
        case Err(e):
            assert False, e
        case Ok(s):
            state: NodeState = s.node_states[0]  # type: ignore
    assert state.state == NodeState.RUNNING

    foo_service.set_service_block(False)

    time.sleep(0.1)

    run_result = tree_control_node.execute_tree(ControlTreeExecution.Request.TICK_ONCE)
    assert run_result.is_ok()

    match tree_control_node.get_tree_state():
        case Err(e):
            assert False, e
        case Ok(s):
            state: NodeState = s.node_states[0]  # type: ignore
    assert state.state == NodeState.SUCCEED


@pytest.mark.launch(fixture=sim_time_tree_node)
@pytest.mark.dependency(depends=["test_send_request"])
def test_fail_timeout(
    tree_control_node: TreeControlNode,
    time_control_node: TimeControlNode,
    foo_service: FooService,
):
    test_send_request(
        tree_control_node,
        time_control_node,
        foo_service,
    )

    run_result = tree_control_node.execute_tree(ControlTreeExecution.Request.TICK_ONCE)
    assert run_result.is_ok()

    match tree_control_node.get_tree_state():
        case Err(e):
            assert False, e
        case Ok(s):
            state: NodeState = s.node_states[0]  # type: ignore
    assert state.state == NodeState.RUNNING

    time_control_node.set_time(15)

    time.sleep(0.1)

    run_result = tree_control_node.execute_tree(ControlTreeExecution.Request.TICK_ONCE)
    assert run_result.is_ok()

    match tree_control_node.get_tree_state():
        case Err(e):
            assert False, e
        case Ok(s):
            state: NodeState = s.node_states[0]  # type: ignore
    assert state.state == NodeState.FAIL


# This marker name can be used for other tests to depend on,
#   in case they rely on this node to work properly.
# NOTE The dependencies for this test should be set in a way
#   that includes all tests in the module.
@pytest.mark.dependency(
    name="Service",
    depends=[
        "test_succeed_response",
        "test_fail_timeout",
    ],
)
def test_confirm_node_function():
    # This test is purely for dependency handling and doesn't do anything on its own.
    pass
