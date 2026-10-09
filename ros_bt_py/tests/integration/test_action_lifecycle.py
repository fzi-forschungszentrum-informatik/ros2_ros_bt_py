# Copyright 2026 FZI Forschungszentrum Informatik
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Integration tests for the Action node lifecycle against a real action server.

These prove, without mocks, that
* a goal the server accepts after untick is canceled instead of left running,
* shutdown bounds its cancel wait with an Event instead of spinning the
  global executor, and
* an unresolved cancel keeps the client alive so shutdown can be retried.
"""
import threading
import time
import unittest.mock as mock

import pytest
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor

from example_interfaces.action import Fibonacci

from ros_bt_py.helpers import BTNodeState
from ros_bt_py.ros_nodes.action import Action, ActionStates
from ros_bt_py.custom_types import RosActionName, RosActionType

from ros_bt_py_interfaces.msg import NodeState

GOAL_ACCEPT_DELAY_S = 0.3
ACTION_NAME = "/test_bt_action_lifecycle"


@pytest.fixture
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def fibonacci_server(ros_context):
    """Real Fibonacci action server with delayed goal acceptance."""
    events = {
        "accepted": threading.Event(),
        "canceled": threading.Event(),
    }
    server_node = rclpy.create_node("test_fibonacci_server_node")

    def goal_callback(goal_request):
        # Delay the acceptance so the client unticks before the answer arrives
        time.sleep(GOAL_ACCEPT_DELAY_S)
        events["accepted"].set()
        return GoalResponse.ACCEPT

    def cancel_callback(goal_handle):
        return CancelResponse.ACCEPT

    def execute_callback(goal_handle):
        deadline = time.monotonic() + 10.0
        while not goal_handle.is_cancel_requested and time.monotonic() < deadline:
            time.sleep(0.05)
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            events["canceled"].set()
        return Fibonacci.Result()

    server = ActionServer(
        server_node,
        Fibonacci,
        ACTION_NAME,
        execute_callback=execute_callback,
        goal_callback=goal_callback,
        cancel_callback=cancel_callback,
        callback_group=ReentrantCallbackGroup(),
    )
    executor = MultiThreadedExecutor()
    executor.add_node(server_node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    yield events
    executor.shutdown()
    spin_thread.join(timeout=5.0)
    server.destroy()
    server_node.destroy_node()


@pytest.fixture
def bt_action_node(ros_context, logging_mock):
    """An Action BT node on its own node, spun by a dedicated executor."""
    client_node = rclpy.create_node("test_bt_action_client_node")
    executor = SingleThreadedExecutor()
    executor.add_node(client_node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    action_node = Action(
        options={
            "action_name": RosActionName(ACTION_NAME),
            "action_type": RosActionType("example_interfaces/action/Fibonacci"),
            "wait_for_action_server_seconds": 10.0,
            "timeout_seconds": 5.0,
            "fail_if_not_available": True,
        },
        ros_node=client_node,
        logging_manager=logging_mock,
    )
    yield action_node
    if action_node.state != BTNodeState.SHUTDOWN:
        action_node.shutdown()
    executor.shutdown()
    spin_thread.join(timeout=5.0)
    client_node.destroy_node()


class TestActionLifecycle:
    def test_untick_cancels_goal_accepted_after_untick(
        self, fibonacci_server, bt_action_node
    ):
        action_node = bt_action_node
        assert action_node.setup().is_ok()
        action_node.inputs["order"] = 3

        # First tick sends the goal; the server's answer is still on its way
        assert action_node.tick().is_ok()
        assert action_node.state == NodeState.RUNNING

        assert action_node.untick().is_ok()
        assert action_node.state == NodeState.IDLE

        # The server accepts the goal after the node stopped waiting for it;
        # the node must cancel it instead of leaving it running
        assert fibonacci_server["canceled"].wait(timeout=10.0)

        # Shutdown waits for the late cancel and releases the client
        assert action_node.shutdown().is_ok()
        assert action_node._ac is None

    def test_shutdown_cancel_wait_is_bounded_and_executor_free(
        self, fibonacci_server, bt_action_node
    ):
        action_node = bt_action_node
        assert action_node.setup().is_ok()
        action_node.inputs["order"] = 3

        # First tick sends the goal, second moves to WAITING_FOR_ACTION_COMPLETE
        assert action_node.tick().is_ok()
        deadline = time.monotonic() + 5.0
        while not (
            action_node._new_goal_request_future
            and action_node._new_goal_request_future.done()
        ):
            assert time.monotonic() < deadline, "goal request never completed"
            time.sleep(0.05)
        assert action_node.tick().is_ok()
        assert action_node._internal_state == ActionStates.WAITING_FOR_ACTION_COMPLETE
        assert action_node.state == NodeState.RUNNING

        start_time = time.monotonic()
        with mock.patch("rclpy.spin_until_future_complete") as spin_mock:
            result = action_node.shutdown()
        elapsed = time.monotonic() - start_time

        # The wait is bounded wall-clock time, not a global-executor spin
        spin_mock.assert_not_called()
        assert result.is_ok()
        assert elapsed < 5.0
        assert action_node._ac is None
