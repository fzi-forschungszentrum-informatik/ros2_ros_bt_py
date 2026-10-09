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
import threading
import time

import pytest
import unittest.mock as mock

from example_interfaces.action import Fibonacci
from ros_bt_py.node import define_bt_node
from ros_bt_py.node_config import NodeConfig
from ros_bt_py.ros_nodes.action import Action, ActionForSetType, ActionStates
from rclpy.task import Future as RclpyFuture
from rclpy.time import Time
from ros_bt_py_interfaces.msg import NodeState, UtilityBounds
from ros_bt_py.exceptions import BehaviorTreeException
from ros_bt_py.custom_types import RosActionName, RosActionType


@define_bt_node(
    NodeConfig(
        options={},
        inputs={},
        outputs={"feedback": object, "result": object},
        max_children=0,
    )
)
class FibonacciActionForSetType(ActionForSetType):
    """Concrete ActionForSetType used to test the abstract variant."""

    def set_action_attributes(self):
        self._action_type = Fibonacci
        self._goal_type = Fibonacci.Goal
        self._feedback_type = Fibonacci.Feedback
        self._result_type = Fibonacci.Result
        self._action_name = self.options["action_name"].name

    def set_goal(self):
        self._input_goal = self._goal_type()

    def set_outputs(self):
        self.outputs["result"] = self._result
        return True

    def set_output_none(self):
        self.outputs["feedback"] = None
        self.outputs["result"] = None


class TestAction:
    @pytest.fixture
    def action_node_no_ros(self, logging_mock):
        action_node = Action(
            options={
                "action_name": RosActionName("this_service_does_not_exist"),
                "action_type": RosActionType("example_interfaces/action/Fibonacci"),
                "wait_for_action_server_seconds": 5.0,
                "timeout_seconds": 5.0,
                "fail_if_not_available": True,
            },
            ros_node=None,
            logging_manager=logging_mock,
        )
        yield action_node

    @pytest.fixture
    def setup_mocks(self, logging_mock):
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
                    "action_name": RosActionName("this_service_does_not_exist"),
                    "action_type": RosActionType("example_interfaces/action/Fibonacci"),
                    "wait_for_action_server_seconds": 5.0,
                    "timeout_seconds": 5.0,
                    "fail_if_not_available": True,
                },
                ros_node=ros_mock,
                logging_manager=logging_mock,
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

    def test_node_failure(self, setup_mocks, warn_log):
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
        with pytest.warns(warn_log, match=".*[cC]ancel.*"):
            action_node.tick()
        ac_instance_mock.send_goal_async.assert_called_with(
            goal=goal, feedback_callback=action_node._feedback_cb
        )
        assert action_node.state == NodeState.FAILED

        feedback_cb_mock.assert_called_once_with(feedback_mock)

    @mock.patch("rclpy.task.Future")
    def test_node_timeout(self, cancel_goal_future_mock, setup_mocks, warn_log):
        action_node = setup_mocks["action_node"]
        running_goal_future_mock = setup_mocks["running_goal_future_mock"]
        ac_instance_mock = setup_mocks["ac_instance_mock"]
        feedback_cb_mock = setup_mocks["feedback_cb_mock"]
        clock_mock = setup_mocks["clock_mock"]
        running_goal_handle_mock = setup_mocks["running_goal_handle_mock"]
        goal_result = setup_mocks["goal_result"]

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
        with pytest.warns(warn_log, match=".*[cC]ancel.*"):
            action_node.tick()
        # requests goal canceling, so node is still running
        assert action_node.state == NodeState.RUNNING
        assert running_goal_handle_mock.cancel_goal_async.called

        # The server's cancel ACK is not terminal: the node keeps the goal's
        # result future and stays running instead of dropping it
        action_node.tick()
        assert action_node.state == NodeState.RUNNING
        assert action_node._running_goal_future is running_goal_future_mock

        # The remote result arrives and is retained in the outputs
        running_goal_future_mock.done.return_value = True
        action_node.tick()
        assert action_node.state == NodeState.SUCCEEDED
        assert action_node.outputs["result.sequence"] == list(
            goal_result.result.sequence
        )

        feedback_cb_mock.assert_called_once_with(feedback_mock)

    def test_untick_cancels_goal_accepted_after_untick(self, setup_mocks):
        """A goal the server accepts after untick must be canceled, not left running."""
        action_node = setup_mocks["action_node"]
        running_goal_handle_mock = setup_mocks["running_goal_handle_mock"]
        self.node_setup(action_node)

        # Goal request is pending, the server has not answered yet
        goal_request_future = RclpyFuture()
        action_node._internal_state = ActionStates.WAITING_FOR_GOAL_ACCEPTANCE
        action_node._new_goal_request_future = goal_request_future

        action_node.untick()
        assert action_node.state == NodeState.IDLE
        assert action_node._new_goal_request_future is None

        # The server accepts the goal after the node stopped waiting for it
        goal_request_future.set_result(running_goal_handle_mock)
        running_goal_handle_mock.cancel_goal_async.assert_called_once()

        # The late cancel is retained so a later shutdown can wait for it
        assert action_node._shutdown_cancel_future is (
            running_goal_handle_mock.cancel_goal_async.return_value
        )

        # A new tick cycle must not inherit the stale goal: the node sends a
        # fresh goal request and tracks only that one
        action_node.inputs["order"] = 2
        assert action_node.tick().is_ok()
        assert action_node._new_goal_request_future is not goal_request_future

    def test_node_reset_shutdown(self, setup_mocks):
        action_node = setup_mocks["action_node"]
        ac_instance_mock = setup_mocks["ac_instance_mock"]
        running_goal_future_mock = setup_mocks["running_goal_future_mock"]
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
        ac_instance_mock.destroy.assert_called_once()

        feedback_cb_mock.assert_called_once_with(feedback_mock)

    def test_shutdown_wait_is_bounded_and_executor_free(self, setup_mocks):
        """Shutdown must bound its cancel wait with an Event on the future,
        never by spinning the global executor."""
        action_node = setup_mocks["action_node"]
        ac_instance_mock = setup_mocks["ac_instance_mock"]
        running_goal_handle_mock = setup_mocks["running_goal_handle_mock"]
        self.node_setup(action_node)

        cancel_future = RclpyFuture()
        running_goal_handle_mock.cancel_goal_async.return_value = cancel_future

        action_node._internal_state = ActionStates.WAITING_FOR_ACTION_COMPLETE
        action_node._running_goal_handle = running_goal_handle_mock

        # The cancel resolves from another thread, as a spinning executor would
        resolve_timer = threading.Timer(
            0.1, lambda: cancel_future.set_result("canceled")
        )
        resolve_timer.start()
        start_time = time.monotonic()
        with mock.patch("rclpy.spin_until_future_complete") as spin_mock:
            result = action_node.shutdown()
        elapsed = time.monotonic() - start_time
        resolve_timer.join(1)

        spin_mock.assert_not_called()
        assert result.is_ok()
        ac_instance_mock.destroy.assert_called_once()
        assert elapsed < 2.0

    def test_shutdown_unresolved_cancel_keeps_client_for_retry(self, setup_mocks):
        """An unresolved goal cancel must not destroy the action client.

        Shutdown returns an error so the caller can retry once the cancel
        resolves; a retry must then complete and destroy the client."""
        action_node = setup_mocks["action_node"]
        ac_instance_mock = setup_mocks["ac_instance_mock"]
        running_goal_handle_mock = setup_mocks["running_goal_handle_mock"]
        self.node_setup(action_node)

        cancel_future = RclpyFuture()
        running_goal_handle_mock.cancel_goal_async.return_value = cancel_future

        action_node._internal_state = ActionStates.WAITING_FOR_ACTION_COMPLETE
        action_node._running_goal_handle = running_goal_handle_mock

        with mock.patch(
            "ros_bt_py.ros_nodes.action._SHUTDOWN_CANCEL_TIMEOUT_S", 0.2
        ), mock.patch("rclpy.spin_until_future_complete"):
            first_result = action_node.shutdown()

        assert first_result.is_err()
        ac_instance_mock.destroy.assert_not_called()
        assert action_node._ac is ac_instance_mock

        # Once the cancel has resolved, a retried shutdown completes cleanly
        cancel_future.set_result("canceled")
        second_result = action_node.shutdown()
        assert second_result.is_ok()
        ac_instance_mock.destroy.assert_called_once()
        assert action_node._ac is None

    def test_shutdown_retries_rejected_goal_cancellation(self, setup_mocks):
        action_node = setup_mocks["action_node"]
        ac_instance_mock = setup_mocks["ac_instance_mock"]
        running_goal_handle_mock = setup_mocks["running_goal_handle_mock"]
        running_goal_future_mock = setup_mocks["running_goal_future_mock"]
        self.node_setup(action_node)

        rejected_cancel = RclpyFuture()
        rejected_response = mock.Mock(goals_canceling=[])
        rejected_cancel.set_result(rejected_response)
        accepted_cancel = RclpyFuture()
        accepted_cancel.set_result("accepted")
        running_goal_handle_mock.cancel_goal_async.side_effect = [
            rejected_cancel,
            accepted_cancel,
        ]
        running_goal_future_mock.done.return_value = False
        action_node._internal_state = ActionStates.WAITING_FOR_ACTION_COMPLETE
        action_node._running_goal_handle = running_goal_handle_mock
        action_node._running_goal_future = running_goal_future_mock

        first_result = action_node.shutdown()

        assert first_result.is_err()
        assert "rejected" in str(first_result.unwrap_err()).lower()
        ac_instance_mock.destroy.assert_not_called()

        with mock.patch("ros_bt_py.ros_nodes.action._SHUTDOWN_CANCEL_TIMEOUT_S", 0.05):
            second_result = action_node.shutdown()
        assert second_result.is_err()
        assert running_goal_handle_mock.cancel_goal_async.call_count == 2

        running_goal_future_mock.done.return_value = True
        assert action_node.shutdown().is_ok()
        ac_instance_mock.destroy.assert_called_once()

    def test_failed_retention_is_consumed_on_next_tick(self, setup_mocks):
        """An internally failed goal retention must not pin the node in RUNNING."""
        action_node = setup_mocks["action_node"]
        ac_instance_mock = setup_mocks["ac_instance_mock"]
        self.node_setup(action_node)

        failed_request = RclpyFuture()
        failed_request.set_exception(RuntimeError("acceptance exploded"))
        action_node._shutdown_goal_request_future = failed_request

        self.create_and_set_input_goal(action_node)
        first_tick = action_node.tick()

        assert first_tick.is_ok()
        assert action_node._shutdown_cleanup_error is None
        ac_instance_mock.send_goal_async.assert_called_once()

    def test_stale_cleanup_error_does_not_block_shutdown_retry(self, setup_mocks):
        """A shutdown that failed on a retention error must stay retryable."""
        action_node = setup_mocks["action_node"]
        ac_instance_mock = setup_mocks["ac_instance_mock"]
        self.node_setup(action_node)

        failed_request = RclpyFuture()
        failed_request.set_exception(RuntimeError("acceptance exploded"))
        action_node._shutdown_goal_request_future = failed_request

        first_result = action_node.shutdown()
        assert first_result.is_err()
        assert "acceptance exploded" in str(first_result.unwrap_err())
        ac_instance_mock.destroy.assert_not_called()

        second_result = action_node.shutdown()
        assert second_result.is_ok()
        ac_instance_mock.destroy.assert_called_once()
        assert action_node._ac is None

    def test_rejected_cancel_at_runtime_waits_for_terminal_result(self, setup_mocks):
        """A cancel rejected because the goal terminated concurrently must fall
        through to waiting for the terminal result instead of erroring the tree."""
        action_node = setup_mocks["action_node"]
        running_goal_handle_mock = setup_mocks["running_goal_handle_mock"]
        running_goal_future_mock = setup_mocks["running_goal_future_mock"]
        self.node_setup(action_node)

        rejected_cancel = RclpyFuture()
        rejected_response = mock.Mock(goals_canceling=[])
        rejected_cancel.set_result(rejected_response)
        running_goal_handle_mock.cancel_goal_async.return_value = rejected_cancel

        action_node._internal_state = ActionStates.WAITING_FOR_GOAL_CANCELLATION
        action_node._cancel_goal_future = rejected_cancel
        action_node._running_goal_handle = running_goal_handle_mock
        action_node._running_goal_future = running_goal_future_mock

        result = action_node._do_tick_wait_for_cancel_complete()

        assert result.is_ok()
        assert action_node._internal_state == ActionStates.WAITING_FOR_ACTION_COMPLETE
        assert action_node._goal_cancel_requested is False

    def test_shutdown_retry_bounds_cancel_goal_exceptions(self, setup_mocks):
        """A cancel_goal_async that raises during the shutdown retry must be a
        bounded error, never an uncaught exception."""
        action_node = setup_mocks["action_node"]
        running_goal_handle_mock = setup_mocks["running_goal_handle_mock"]
        running_goal_future_mock = setup_mocks["running_goal_future_mock"]
        self.node_setup(action_node)

        running_goal_future_mock.done.return_value = False
        action_node._shutdown_goal_handle = running_goal_handle_mock
        action_node._shutdown_result_future = running_goal_future_mock
        running_goal_handle_mock.cancel_goal_async.side_effect = RuntimeError(
            "cancel exploded"
        )

        result = action_node.shutdown()

        assert result.is_err()
        assert "cancel exploded" in str(result.unwrap_err())

    def test_shutdown_cancels_a_retained_goal_without_result_future(self, setup_mocks):
        """A retained goal handle whose result future is missing must still be
        cancelled before the client is destroyed."""
        action_node = setup_mocks["action_node"]
        running_goal_handle_mock = setup_mocks["running_goal_handle_mock"]
        ac_instance_mock = setup_mocks["ac_instance_mock"]
        self.node_setup(action_node)

        # Retention lost the result future (e.g. get_result_async() raised)
        action_node._shutdown_goal_handle = running_goal_handle_mock
        action_node._shutdown_result_future = None
        cancel_future = RclpyFuture()
        running_goal_handle_mock.cancel_goal_async.return_value = cancel_future
        cancel_future.set_result("canceled")

        result = action_node.shutdown()

        assert result.is_ok()
        running_goal_handle_mock.cancel_goal_async.assert_called_once()
        ac_instance_mock.destroy.assert_called_once()

    def test_send_new_goal_errors_when_client_is_not_initialized(self, setup_mocks):
        """Matches ActionForSetType: an uninitialized client is a hard error, not BROKEN."""
        action_node = setup_mocks["action_node"]
        self.node_setup(action_node)
        action_node._ac = None

        result = action_node._do_tick_send_new_goal()

        assert result.is_err()

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

    def test_node_no_ros(self, action_node_no_ros, error_log):
        assert action_node_no_ros is not None
        with pytest.warns(error_log):
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
            "feedback.sequence" in action_node.outputs
            and "result.sequence" in action_node.outputs
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

        assert action_node.outputs["feedback.sequence"] == list(
            feedback_mock.feedback.sequence
        )
        assert isinstance(action_node.outputs["feedback.sequence"], list)
        assert action_node.outputs["result.sequence"] is None

        # Result available
        running_goal_future_mock.done.return_value = True
        action_node.tick()
        assert action_node.state == NodeState.SUCCEEDED
        assert isinstance(action_node.outputs["result.sequence"], list)
        assert action_node.outputs["result.sequence"] == list(
            goal_result.result.sequence
        )

        feedback_cb_mock.assert_called_once_with(feedback_mock)


class TestActionForSetType:
    @pytest.fixture
    def setup_mocks_set_type(self, logging_mock):
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

            client_mock.return_value = ac_instance_mock
            ac_instance_mock.wait_for_server.return_value = True
            ac_instance_mock.send_goal_async.return_value = new_goal_request_future_mock

            new_goal_request_future_mock.result.return_value = running_goal_handle_mock
            new_goal_request_future_mock.done.return_value = True

            running_goal_handle_mock.get_result_async.return_value = (
                running_goal_future_mock
            )
            running_goal_future_mock.cancelled.return_value = False

            clock_mock.now.return_value = Time(seconds=0)
            ros_mock.get_clock.return_value = clock_mock

            action_node = FibonacciActionForSetType(
                options={
                    "action_name": RosActionName("this_service_does_not_exist"),
                    "wait_for_action_server_seconds": 5.0,
                    "timeout_seconds": 5.0,
                },
                ros_node=ros_mock,
                logging_manager=logging_mock,
            )

            yield {
                "action_node": action_node,
                "ros_mock": ros_mock,
                "clock_mock": clock_mock,
                "new_goal_request_future_mock": new_goal_request_future_mock,
                "running_goal_handle_mock": running_goal_handle_mock,
                "running_goal_future_mock": running_goal_future_mock,
                "client_mock": client_mock,
                "ac_instance_mock": ac_instance_mock,
            }

    def node_setup(self, action_node):
        assert action_node is not None
        action_node.setup()
        assert action_node.state == NodeState.IDLE

    def test_untick_cancels_goal_accepted_after_untick(self, setup_mocks_set_type):
        """A goal the server accepts after untick must be canceled, not left running."""
        action_node = setup_mocks_set_type["action_node"]
        running_goal_handle_mock = setup_mocks_set_type["running_goal_handle_mock"]
        self.node_setup(action_node)

        goal_request_future = RclpyFuture()
        action_node._internal_state = ActionStates.WAITING_FOR_GOAL_ACCEPTANCE
        action_node._new_goal_request_future = goal_request_future

        action_node.untick()
        assert action_node.state == NodeState.IDLE
        assert action_node._new_goal_request_future is None

        goal_request_future.set_result(running_goal_handle_mock)
        running_goal_handle_mock.cancel_goal_async.assert_called_once()
        assert action_node._shutdown_cancel_future is (
            running_goal_handle_mock.cancel_goal_async.return_value
        )

    def test_shutdown_unresolved_cancel_keeps_client_for_retry(
        self, setup_mocks_set_type
    ):
        """An unresolved goal cancel must not destroy the action client."""
        action_node = setup_mocks_set_type["action_node"]
        ac_instance_mock = setup_mocks_set_type["ac_instance_mock"]
        running_goal_handle_mock = setup_mocks_set_type["running_goal_handle_mock"]
        self.node_setup(action_node)

        cancel_future = RclpyFuture()
        running_goal_handle_mock.cancel_goal_async.return_value = cancel_future

        action_node._internal_state = ActionStates.WAITING_FOR_ACTION_COMPLETE
        action_node._running_goal_handle = running_goal_handle_mock

        with mock.patch(
            "ros_bt_py.ros_nodes.action._SHUTDOWN_CANCEL_TIMEOUT_S", 0.2
        ), mock.patch("rclpy.spin_until_future_complete"):
            first_result = action_node.shutdown()

        assert first_result.is_err()
        ac_instance_mock.destroy.assert_not_called()
        assert action_node._ac is ac_instance_mock

        cancel_future.set_result("canceled")
        second_result = action_node.shutdown()
        assert second_result.is_ok()
        ac_instance_mock.destroy.assert_called_once()
        assert action_node._ac is None

    def test_cancel_ack_retains_remote_result(self, setup_mocks_set_type):
        """The cancel ACK is not terminal: wait for and retain the remote result."""
        action_node = setup_mocks_set_type["action_node"]
        running_goal_future_mock = setup_mocks_set_type["running_goal_future_mock"]
        running_goal_handle_mock = setup_mocks_set_type["running_goal_handle_mock"]
        clock_mock = setup_mocks_set_type["clock_mock"]
        self.node_setup(action_node)

        cancel_future = RclpyFuture()
        running_goal_handle_mock.cancel_goal_async.return_value = cancel_future
        running_goal_future_mock.done.return_value = False

        # Goal accepted, running
        action_node.tick()
        assert action_node.state == NodeState.RUNNING

        # Timeout: request goal cancellation
        clock_mock.now.return_value = Time(seconds=10)
        action_node.tick()
        assert running_goal_handle_mock.cancel_goal_async.called

        # Cancel ACK: not terminal, the result future is retained
        cancel_future.set_result("canceled")
        action_node.tick()
        assert action_node.state == NodeState.RUNNING
        assert action_node._running_goal_future is running_goal_future_mock

        # Remote result arrives and is retained in the outputs
        running_goal_future_mock.done.return_value = True
        goal_result = mock.Mock()
        running_goal_future_mock.result.return_value = goal_result
        action_node.tick()
        assert action_node.state == NodeState.SUCCEEDED
        assert action_node.outputs["result"] is goal_result
