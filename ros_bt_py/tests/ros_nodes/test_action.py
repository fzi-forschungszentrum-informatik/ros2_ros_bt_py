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
from typing import Any, Generator
import pytest

import unittest.mock as mock
from example_interfaces.action import Fibonacci
from ros_bt_py.ros_nodes.action import Action
from rclpy.time import Time
from ros_bt_py_interfaces.msg import NodeState, UtilityBounds
from ros_bt_py.exceptions import BehaviorTreeException


class TestAction:
    @pytest.fixture
    def action_node_no_ros(self):
        action_node = Action(
            options={
                "action_name": "this_service_does_not_exist",
                "action_type": Fibonacci,
                "wait_for_action_server_seconds": 5.0,
                "timeout_seconds": 5.0,
                "fail_if_not_available": True,
            },
            ros_node=None,
        )
        yield action_node

    @pytest.fixture
    def setup_mocks(self):
        ac_instance_mock = mock.Mock()
        with mock.patch("rclpy.node.Node") as ros_mock, mock.patch(
            "ros_bt_py.ros_nodes.action.ActionClient"
        ) as client_mock, mock.patch(
            "rclpy.task.Future"
        ) as new_goal_request_future_mock, mock.patch(
            "rclpy.action.client.ClientGoalHandle"
        ) as running_goal_handle_mock, mock.patch(
            "rclpy.task.Future"
        ) as running_goal_future_mock, mock.patch(
            "rclpy.clock.Clock"
        ) as clock_mock:

            ac_instance_mock = mock.Mock()
            client_mock.return_value = ac_instance_mock
            ac_instance_mock.wait_for_server.return_value = True
            ac_instance_mock.send_goal_async.return_value = new_goal_request_future_mock

            new_goal_request_future_mock.result.return_value = running_goal_handle_mock
            new_goal_request_future_mock.done.return_value = True

            running_goal_handle_mock.get_result_async.return_value = (
                running_goal_future_mock
            )

            running_goal_future_mock.cancelled.return_value = False

            clock_mock.now.side_effect = [
                Time(seconds=0),
                Time(seconds=1),
                Time(seconds=2),
                Time(seconds=3),
                Time(seconds=4),
            ]
            ros_mock.get_clock.return_value = clock_mock

            goal_result = mock.Mock()
            running_goal_future_mock.result.return_value = goal_result
            result = Fibonacci.Result()
            goal_result.result = result

            action_node = Action(
                options={
                    "action_name": "this_service_does_not_exist",
                    "action_type": Fibonacci,
                    "wait_for_action_server_seconds": 5.0,
                    "timeout_seconds": 5.0,
                    "fail_if_not_available": True,
                },
                ros_node=ros_mock,
            )

            feedback_cb_patcher = mock.patch.object(
                action_node, "_feedback_cb", wraps=action_node._feedback_cb  # type: ignore
            )
            feedback_cb_mock = feedback_cb_patcher.start()

            yield {
                "action_node": action_node,
                "feedback_cb_mock": feedback_cb_mock,
                "ros_mock": ros_mock,
                "clock_mock": clock_mock,
                "new_goal_request_future_mock": new_goal_request_future_mock,
                "running_goal_handle_mock": running_goal_handle_mock,
                "running_goal_future_mock": running_goal_future_mock,
                "client_mock": client_mock,
                "ac_instance_mock": ac_instance_mock,
                "goal_result": goal_result,
            }
            feedback_cb_patcher.stop()

    def node_setup(self, action_node):
        assert action_node is not None
        action_node.setup()
        assert action_node.state == NodeState.IDLE

    def create_and_simulate_feedback(self, action_node, sequence=[0]):
        feedback_mock = mock.Mock()
        feedback_mock.feedback = Fibonacci.Feedback()
        feedback_mock.feedback.sequence = sequence
        action_node._feedback_cb(feedback_mock)
        return feedback_mock

    def create_and_set_input_goal(self, action_node, order=2):
        goal = Fibonacci.Goal()
        goal.order = order
        action_node.inputs["order"] = goal.order
        return goal

    def test_node_success(self, setup_mocks):
        action_node = setup_mocks["action_node"]
        running_goal_future_mock = setup_mocks["running_goal_future_mock"]
        ac_instance_mock = setup_mocks["ac_instance_mock"]
        feedback_cb_mock = setup_mocks["feedback_cb_mock"]

        self.node_setup(action_node)
        goal = self.create_and_set_input_goal(action_node)
        feedback_mock = self.create_and_simulate_feedback(action_node)

        # Waiting for result
        running_goal_future_mock.done.return_value = False
        action_node.tick()
        ac_instance_mock.send_goal_async.assert_called_with(
            goal=goal, feedback_callback=action_node._feedback_cb
        )
        assert action_node.state == NodeState.RUNNING

        # Result available
        running_goal_future_mock.done.return_value = True
        action_node.tick()
        assert action_node.state == NodeState.SUCCEEDED

        action_node.shutdown()
        assert action_node.state == NodeState.SHUTDOWN

        feedback_cb_mock.assert_called_once_with(feedback_mock)

    def test_node_failure(self, setup_mocks):
        action_node = setup_mocks["action_node"]
        running_goal_future_mock = setup_mocks["running_goal_future_mock"]
        ac_instance_mock = setup_mocks["ac_instance_mock"]
        feedback_cb_mock = setup_mocks["feedback_cb_mock"]

        # Cause of failure
        running_goal_future_mock.cancelled.return_value = True

        self.node_setup(action_node)
        goal = self.create_and_set_input_goal(action_node)
        feedback_mock = self.create_and_simulate_feedback(action_node)

        running_goal_future_mock.done.return_value = False
        action_node.tick()
        ac_instance_mock.send_goal_async.assert_called_with(
            goal=goal, feedback_callback=action_node._feedback_cb
        )
        assert action_node.state == NodeState.FAILED

        feedback_cb_mock.assert_called_once_with(feedback_mock)

    @mock.patch("rclpy.task.Future")
    def test_node_timeout(self, cancel_goal_future_mock, setup_mocks):
        action_node = setup_mocks["action_node"]
        running_goal_future_mock = setup_mocks["running_goal_future_mock"]
        ac_instance_mock = setup_mocks["ac_instance_mock"]
        feedback_cb_mock = setup_mocks["feedback_cb_mock"]
        clock_mock = setup_mocks["clock_mock"]
        running_goal_handle_mock = setup_mocks["running_goal_handle_mock"]

        clock_mock.now.side_effect = None
        clock_mock.now.return_value = Time(seconds=0)
        running_goal_handle_mock.cancel_goal_async.return_value = (
            cancel_goal_future_mock
        )
        cancel_goal_future_mock.done.return_value = True

        self.node_setup(action_node)
        goal = self.create_and_set_input_goal(action_node)
        feedback_mock = self.create_and_simulate_feedback(action_node)

        # Waiting for result
        running_goal_future_mock.done.return_value = False
        action_node.tick()
        ac_instance_mock.send_goal_async.assert_called_with(
            goal=goal, feedback_callback=action_node._feedback_cb
        )
        assert action_node.state == NodeState.RUNNING

        # Set time > timeout_seconds of action_node to create timeout
        clock_mock.now.return_value = Time(seconds=10)
        action_node.tick()
        # requests goal canceling, so node is still running
        assert action_node.state == NodeState.RUNNING

        # Goal gets canceled succesfully
        action_node.tick()
        assert running_goal_future_mock.cancel.called
        assert running_goal_handle_mock.cancel_goal_async.called
        assert action_node.state == NodeState.FAILED

        feedback_cb_mock.assert_called_once_with(feedback_mock)

    def test_node_reset_shutdown(self, setup_mocks):
        action_node = setup_mocks["action_node"]
        running_goal_future_mock = setup_mocks["running_goal_future_mock"]
        ac_instance_mock = setup_mocks["ac_instance_mock"]
        feedback_cb_mock = setup_mocks["feedback_cb_mock"]

        self.node_setup(action_node)
        goal = self.create_and_set_input_goal(action_node)
        feedback_mock = self.create_and_simulate_feedback(action_node)

        # Waiting for result
        running_goal_future_mock.done.return_value = False
        action_node.tick()
        ac_instance_mock.send_goal_async.assert_called_with(
            goal=goal, feedback_callback=action_node._feedback_cb
        )
        assert action_node.state == NodeState.RUNNING

        # Result available
        running_goal_future_mock.done.return_value = True
        action_node.tick()
        assert action_node.state == NodeState.SUCCEEDED

        action_node.reset()
        assert action_node.state == NodeState.IDLE

        action_node.shutdown()
        assert action_node.state == NodeState.SHUTDOWN

        feedback_cb_mock.assert_called_once_with(feedback_mock)

    def test_node_reset(self, setup_mocks):
        action_node = setup_mocks["action_node"]
        running_goal_future_mock = setup_mocks["running_goal_future_mock"]
        ac_instance_mock = setup_mocks["ac_instance_mock"]

        self.node_setup(action_node)
        goal = self.create_and_set_input_goal(action_node)
        self.create_and_simulate_feedback(action_node)

        # Waiting for result
        running_goal_future_mock.done.return_value = False
        action_node.tick()
        ac_instance_mock.send_goal_async.assert_called_with(
            goal=goal, feedback_callback=action_node._feedback_cb
        )
        assert action_node.state == NodeState.RUNNING

        # Result available
        running_goal_future_mock.done.return_value = True
        action_node.tick()
        assert action_node.state == NodeState.SUCCEEDED

        action_node.reset()
        assert action_node.state == NodeState.IDLE
        self.create_and_simulate_feedback(action_node)

        # Waiting for result
        running_goal_future_mock.done.return_value = False
        action_node.tick()
        ac_instance_mock.send_goal_async.assert_called_with(
            goal=goal, feedback_callback=action_node._feedback_cb
        )
        assert action_node.state == NodeState.RUNNING

        # Result available
        running_goal_future_mock.done.return_value = True
        action_node.tick()
        assert action_node.state == NodeState.SUCCEEDED

    def test_node_no_ros(self, action_node_no_ros):
        assert action_node_no_ros is not None
        setup_result = action_node_no_ros.setup()
        assert setup_result.is_err()
        assert isinstance(setup_result.err(), BehaviorTreeException)

    def test_node_untick(self, setup_mocks):
        action_node = setup_mocks["action_node"]
        running_goal_future_mock = setup_mocks["running_goal_future_mock"]
        ac_instance_mock = setup_mocks["ac_instance_mock"]
        feedback_cb_mock = setup_mocks["feedback_cb_mock"]
        running_goal_handle_mock = setup_mocks["running_goal_handle_mock"]

        self.node_setup(action_node)
        goal = self.create_and_set_input_goal(action_node)
        feedback_mock = self.create_and_simulate_feedback(action_node)

        # Waiting for result
        running_goal_future_mock.done.return_value = False
        action_node.tick()
        ac_instance_mock.send_goal_async.assert_called_with(
            goal=goal, feedback_callback=action_node._feedback_cb
        )
        assert action_node.state == NodeState.RUNNING

        action_node.untick()
        assert action_node.state == NodeState.IDLE
        assert running_goal_handle_mock.cancel_goal_async.called

        feedback_cb_mock.assert_called_once_with(feedback_mock)

    def test_node_utility_no_ros(self, setup_mocks, action_node_no_ros):
        action_node = setup_mocks["action_node"]

        bounds_no_ros_result = action_node_no_ros.calculate_utility()
        assert bounds_no_ros_result.ok() == UtilityBounds(can_execute=False)

        bounds_result = action_node.calculate_utility()
        assert bounds_result.ok() == UtilityBounds(can_execute=False)

    def test_node_utility(self, setup_mocks):
        action_node = setup_mocks["action_node"]
        running_goal_future_mock = setup_mocks["running_goal_future_mock"]
        ac_instance_mock = setup_mocks["ac_instance_mock"]
        feedback_cb_mock = setup_mocks["feedback_cb_mock"]

        self.node_setup(action_node)
        goal = self.create_and_set_input_goal(action_node)
        feedback_mock = self.create_and_simulate_feedback(action_node)

        # No result yet
        running_goal_future_mock.done.return_value = False
        action_node.tick()
        ac_instance_mock.send_goal_async.assert_called_with(
            goal=goal, feedback_callback=action_node._feedback_cb
        )
        assert action_node.state == NodeState.RUNNING

        ac_instance_mock.server_is_ready.return_value = False
        bounds_result = action_node.calculate_utility()
        assert bounds_result.ok() == UtilityBounds(can_execute=False)

        ac_instance_mock.server_is_ready.return_value = True
        bounds_result = action_node.calculate_utility()
        assert bounds_result.ok() == UtilityBounds(
            can_execute=True,
            has_lower_bound_success=True,
            has_upper_bound_success=True,
            has_lower_bound_failure=True,
            has_upper_bound_failure=True,
        )
        feedback_cb_mock.assert_called_once_with(feedback_mock)

    def test_node_outputs(self, setup_mocks):
        action_node = setup_mocks["action_node"]
        running_goal_future_mock = setup_mocks["running_goal_future_mock"]
        ac_instance_mock = setup_mocks["ac_instance_mock"]
        feedback_cb_mock = setup_mocks["feedback_cb_mock"]
        goal_result = setup_mocks["goal_result"]

        self.node_setup(action_node)

        assert (
            "feedback_sequence" in action_node.outputs
            and "result_sequence" in action_node.outputs
        )

        goal = self.create_and_set_input_goal(action_node)
        feedback_mock = self.create_and_simulate_feedback(action_node)

        # Waiting for result
        running_goal_future_mock.done.return_value = False
        action_node.tick()
        ac_instance_mock.send_goal_async.assert_called_with(
            goal=goal, feedback_callback=action_node._feedback_cb
        )
        assert action_node.state == NodeState.RUNNING

        assert action_node.outputs["feedback_sequence"] == list(
            feedback_mock.feedback.sequence
        )
        assert isinstance(action_node.outputs["feedback_sequence"], list)
        assert action_node.outputs["result_sequence"] is None

        # Result available
        running_goal_future_mock.done.return_value = True
        action_node.tick()
        assert action_node.state == NodeState.SUCCEEDED
        assert isinstance(action_node.outputs["result_sequence"], list)
        assert action_node.outputs["result_sequence"] == list(
            goal_result.result.sequence
        )

        feedback_cb_mock.assert_called_once_with(feedback_mock)
