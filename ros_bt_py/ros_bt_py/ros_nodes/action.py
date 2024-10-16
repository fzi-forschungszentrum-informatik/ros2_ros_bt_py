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
from threading import Lock
from abc import ABC, abstractmethod
from typing import Optional, Any, Dict
from enum import Enum

import rclpy
from rclpy.action.client import ActionClient, ClientGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.time import Time

from ros_bt_py_interfaces.msg import Node as NodeMsg
from ros_bt_py_interfaces.msg import UtilityBounds

from ros_bt_py.exceptions import BehaviorTreeException
from ros_bt_py.node import Leaf, define_bt_node
from ros_bt_py.node_config import NodeConfig
from rclpy.node import Node
from ros_bt_py.debug_manager import DebugManager
from ros_bt_py.subtree_manager import SubtreeManager
import inspect
from std_msgs.msg import Int64


class ActionStates(Enum):
    IDLE = 0
    WAITING_FOR_GOAL_ACCEPTANCE = 1
    WAITING_FOR_ACTION_COMPLETE = 2
    REQUEST_GOAL_CANCELLATION = 3
    WAITING_FOR_GOAL_CANCELLATION = 4
    FINISHED = 5


@define_bt_node(
    NodeConfig(
        options={
            "action_name": str,
            "wait_for_action_server_seconds": float,
            "timeout_seconds": float,
            "fail_if_not_available": bool,
        },
        inputs={},
        outputs={},
        max_children=0,
        optional_options=["fail_if_not_available"],
    )
)
class ActionForSetType(ABC, Leaf):
    """
    Abstract ROS action class.

    This class can be inherited to create ROS action nodes with a defined action type.
    Supports building simple custom nodes.

    Will always return RUNNING on the tick a new goal is sent, even if
    the server replies really quickly!

    On every tick, outputs['feedback'] and outputs['result'] (if
    available) are updated.

    On untick, reset or shutdown, the goal is cancelled and will be
    re-sent on the next tick.

    Example:
    -------
        >>> @define_bt_node(NodeConfig(
                options={'MyOption': MyOptionsType},
                inputs={'MyInput': MyInputType},
                outputs={'MyOutput': MyOutputType}, # feedback, goal_status, result,..
                max_children=0))
        >>> class MyActionClass(ActionForSetType):
                # set all important action attributes
                def set_action_attributes(self):
                    self._action_type = MyAction
                    self._goal_type = MyActionGoal
                    self._feedback_type = MyActionFeedback
                    self._result_type = MyActionResult

                    self._action_name = self.options['MyAction']

                # set the action goal
                def set_goal(self):
                    self._input_goal = MyActionGoal()
                    self._input_goal.MyInput = self.inputs['MyImput']
                # overwrite, if there is more than one output key to be overwritten
                def set_output_none(self):
                    self.outputs["feedback"] = None
                    self.outputs["result"] = None
                # set result
                # Return True if SUCCEEDED, False if FAILED
                def set_outputs(self):
                    self.outputs["OUTPUT_KEY"] = self._result.result
                    return "TRUTHVALUE"

    """

    _internal_state = ActionStates.IDLE
    """Internal state of the action."""

    _new_goal_request_future: Optional[rclpy.Future] = None
    """Future for requesting a new goal to be executed."""

    _running_goal_handle: Optional[ClientGoalHandle] = None
    """Goal handle for the currently running goal!."""

    _running_goal_future: Optional[rclpy.Future] = None
    """Future on the current goal handle."""

    _cancel_goal_future: Optional[rclpy.Future] = None
    """Future to request the cancellation of the goal."""

    _action_goal: Optional[Any] = None

    _action_available: bool = True

    @abstractmethod
    def set_action_attributes(self):
        """Set all important action attributes."""
        self._action_type = "ENTER_ACTION_TYPE"
        self._goal_type = "ENTER_GOAL_TYPE"
        self._feedback_type = "ENTER_FEEDBACK_TYPE"
        self._result_type = "ENTER_RESULT_TYPE"

        self._action_name = self.options["action_name"]

    def set_input(self):
        pass

    # overwrite, if there is more than one output key to be overwritten
    def set_output_none(self):
        self.outputs["feedback"] = None
        self.outputs["result"] = None

    @abstractmethod
    def set_goal(self):
        self._input_goal = "ENTER_GOAL_FROM_INPUT"

    # Sets the output (in relation to the result) (define output key while overwriting)
    # Should return True, if the node state should be SUCCEEDED after receiving the message
    # and False, if it's in the FAILED state
    @abstractmethod
    def set_outputs(self):
        self.outputs["OUTPUT_KEY"] = self._result.result
        return "TRUTHVALUE"

    def _do_setup(self):
        if not self.has_ros_node:
            error_msg = f"Node {self.name} does not have a reference to a ROS node!"
            self.logerr(error_msg)
            raise BehaviorTreeException(error_msg)
        self._lock = Lock()
        self._feedback = None
        self._active_goal = None
        self._result = None

        self._internal_state = ActionStates.IDLE

        self._new_goal_request_future = None
        self._running_goal_handle = None
        self._running_goal_future = None

        self._cancel_goal_future = None

        self._action_available = True
        self._shutdown: bool = False

        self.set_action_attributes()
        self._ac = ActionClient(
            node=self.ros_node,
            action_type=self._action_type,
            action_name=self._action_name,
            callback_group=ReentrantCallbackGroup(),
        )

        if not self._ac.wait_for_server(
            timeout_sec=self.options["wait_for_action_server_seconds"]
        ):
            self._action_available = False
            if (
                "fail_if_not_available" not in self.options
                or not self.options["fail_if_not_available"]
            ):
                raise BehaviorTreeException(
                    f"Action server {self._action_name} not available after waiting "
                    f"{self.options['wait_for_action_server_seconds']} seconds!"
                )

        self._last_goal_time: Optional[Time] = None
        self.set_output_none()

        return NodeMsg.IDLE

    def _feedback_cb(self, feedback) -> None:
        self.logdebug(f"Received feedback message: {feedback}")
        with self._lock:
            self._feedback = feedback

    def _do_tick_wait_for_action_complete(self) -> str:
        if self._running_goal_handle is None or self._running_goal_future is None:
            self._internal_state = ActionStates.FINISHED
            return NodeMsg.BROKEN

        if self._running_goal_future.done():
            self._result = self._running_goal_future.result()
            if self._result is None:
                self._running_goal_handle = None
                self._running_goal_future = None
                self._active_goal = None

                self._internal_state = ActionStates.FINISHED

                self.logdebug("Action result is none, action call must have failed!")
                return NodeMsg.FAILED

            # returns failed except the set.ouput() method returns True
            new_state = NodeMsg.FAILED
            if self.set_outputs():
                new_state = NodeMsg.SUCCEEDED
            self._running_goal_handle = None
            self._running_goal_future = None
            self._result = None

            self._internal_state = ActionStates.FINISHED

            self.logdebug("Action succeeded, publishing result!")
            return new_state

        if self._running_goal_future.cancelled():
            self._running_goal_handle = None
            self._running_goal_future = None
            self._active_goal = None

            self._internal_state = ActionStates.FINISHED

            self.logwarn("Action execution was cancelled by the remote server!")
            return NodeMsg.FAILED
        seconds_running = (
            self._running_goal_start_time - self.ros_node.get_clock().now()
        ).nanoseconds / 1e9

        if seconds_running > self.options["timeout_seconds"]:
            self.logwarn(f"Cancelling goal after {seconds_running:f} seconds!")

            # This cancels the goal result future, this is not cancelling the goal.
            self._running_goal_future.cancel()
            self._running_goal_future = None

            self._internal_state = ActionStates.REQUEST_GOAL_CANCELLATION
            return NodeMsg.RUNNING

        return NodeMsg.RUNNING

    def _do_tick_cancel_running_goal(self) -> str:
        if self._running_goal_handle is None:
            self.logwarn(
                "Goal cancellation was requested, but there is no handle to the running goal!"
            )
            self._internal_state = ActionStates.FINISHED
            return NodeMsg.BROKEN

        self._cancel_goal_future = self._running_goal_handle.cancel_goal_async()
        self._internal_state = ActionStates.WAITING_FOR_GOAL_CANCELLATION
        return NodeMsg.SUCCEED

    def _do_tick_wait_for_cancel_complete(self) -> str:
        if self._cancel_goal_future is None:
            self.logwarn(
                "Waiting for goal cancellation to complete, but the future is none!"
            )
            self._internal_state = ActionStates.FINISHED
            return NodeMsg.BROKEN

        if self._cancel_goal_future.done():
            self.logdebug("Successfully cancelled goal exectution!")

            self._cancel_goal_future = None
            self._running_goal_handle = None
            self._running_goal_future = None
            self._active_goal = None

            self._internal_state = ActionStates.FINISHED
            return NodeMsg.SUCCEED
        if self._cancel_goal_future.cancelled():
            self.logdebug("Goal cancellation was cancelled!")

            self._cancel_goal_future = None
            self._running_goal_handle = None
            self._running_goal_future = None
            self._active_goal = None

            self._internal_state = ActionStates.FINISHED
            return NodeMsg.FAILURE
        return NodeMsg.RUNNING

    def _do_tick_send_new_goal(self) -> str:
        """Tick to request the execution of a new goal on the action server."""
        self._new_goal_request_future = self._ac.send_goal_async(
            goal=self._input_goal, feedback_callback=self._feedback_cb
        )

        self._active_goal = self._input_goal
        self._internal_state = ActionStates.WAITING_FOR_GOAL_ACCEPTANCE

        return NodeMsg.SUCCEED

    def _do_tick_wait_for_new_goal_complete(self) -> str:
        """Tick to wait for the new goal to be accepted by the action server!."""
        if self._new_goal_request_future is None:
            self.logerr(
                "Waiting for the goal to be accepted"
                "on the action server, but the future is none!"
            )
            self._internal_state = ActionStates.IDLE
            return NodeMsg.BROKEN

        if self._new_goal_request_future.done():
            self._running_goal_handle = self._new_goal_request_future.result()
            self._new_goal_request_future = None

            if self._running_goal_handle is None:
                self.logwarn("Action goal was rejeced by the server!")
                self._internal_state = ActionStates.FINISHED
                return NodeMsg.FAILED

            self._running_goal_start_time = self.ros_node.get_clock().now()
            self._running_goal_future = self._running_goal_handle.get_result_async()

            self._internal_state = ActionStates.WAITING_FOR_ACTION_COMPLETE
            return NodeMsg.SUCCEED

        if self._new_goal_request_future.cancelled():
            self.logwarn("Request for a new goal was cancelled!")
            self._new_goal_request_future = None
            self._running_goal_handle = None
            self._active_goal = None

            self._internal_state = ActionStates.FINISHED
            return NodeMsg.FAILED

        return NodeMsg.RUNNING

    def _do_tick(self):
        if self.simulate_tick:
            self.logdebug("Simulating tick. Action is not executing!")
            if self.succeed_always:
                return NodeMsg.SUCCEEDED

            return NodeMsg.RUNNING

        if not self._action_available:
            if (
                "fail_if_not_available" in self.options
                and self.options["fail_if_not_available"]
            ):
                return NodeMsg.FAILED

        self.set_input()
        self.set_goal()

        if self._internal_state == ActionStates.IDLE:
            status = self._do_tick_send_new_goal()
            if status not in [NodeMsg.SUCCEED]:
                return status

        if self._internal_state == ActionStates.WAITING_FOR_GOAL_ACCEPTANCE:
            status = self._do_tick_wait_for_new_goal_complete()
            if status not in [NodeMsg.SUCCEED]:
                return status

        if self._internal_state == ActionStates.WAITING_FOR_ACTION_COMPLETE:
            if self._active_goal == self._input_goal:
                return self._do_tick_wait_for_action_complete()
            else:
                # We have a new goal, we should cancel the running one!
                self._internal_state = ActionStates.REQUEST_GOAL_CANCELLATION

        if self._internal_state == ActionStates.REQUEST_GOAL_CANCELLATION:
            status = self._do_tick_cancel_running_goal()
            # Check if goal cancel request was succssful!
            if status not in [NodeMsg.SUCCEED]:
                return status

        if self._internal_state == ActionStates.WAITING_FOR_GOAL_CANCELLATION:
            return self._do_tick_wait_for_cancel_complete()

        if self._internal_state == ActionStates.FINISHED:
            return self._do_tick_finished()

        return NodeMsg.BROKEN

    def _do_tick_finished(self):
        if self._active_goal == self._input_goal:
            return self._state
        else:
            self._internal_state = ActionStates.IDLE
            return NodeMsg.RUNNING

    def _do_untick(self):
        if self._internal_state == ActionStates.WAITING_FOR_ACTION_COMPLETE:
            self._do_tick_cancel_running_goal()

        self._last_goal_time = None
        self._running_goal_future = None
        self._running_goal_handle = None
        self._cancel_goal_future = None
        self._active_goal = None
        self._feedback = None
        self._internal_state = ActionStates.IDLE

        return NodeMsg.IDLE

    def _do_reset(self):
        # same as untick...
        self._do_untick()
        # but also clear the outputs
        self.outputs["feedback"] = None
        self.outputs["result"] = None
        return NodeMsg.IDLE

    def _do_shutdown(self):
        # nothing to do beyond what's done in reset
        self._do_reset()
        self._action_available = False

    def _do_calculate_utility(self):
        if not self.has_ros_node:
            return UtilityBounds(can_execute=False)
        if self._ac is None:
            return UtilityBounds(can_execute=False)
        if not self._ac.server_is_ready():
            return UtilityBounds(can_execute=False)
        return UtilityBounds(
            can_execute=True,
            has_lower_bound_success=True,
            has_upper_bound_success=True,
            has_lower_bound_failure=True,
            has_upper_bound_failure=True,
        )


@define_bt_node(
    NodeConfig(
        version="0.1.0",
        options={
            "action_type": type,
            "action_name": str,
            "wait_for_action_server_seconds": float,
            "timeout_seconds": float,
            "fail_if_not_available": bool,
        },
        inputs={},
        outputs={},
        max_children=0,
        optional_options=["fail_if_not_available"],
    )
)
class Action(Leaf):
    """
    Connect to a ROS action and sends the supplied goal.

    Will always return RUNNING on the tick a new goal is sent, even if
    the server replies really quickly!

    On every tick, outputs['feedback'] and outputs['result'] (if
    available) are updated.

    On untick, reset or shutdown, the goal is cancelled and will be
    re-sent on the next tick.
    """

    _action_name: str
    _goal_type: type
    _feedback_type: type
    _result_type: type
    _ac: Optional[ActionClient] = None
    _feedback = None

    _internal_state = ActionStates.IDLE

    def __init__(
        self,
        options: Optional[Dict] = None,
        debug_manager: Optional[DebugManager] = None,
        subtree_manager: Optional[SubtreeManager] = None,
        name: Optional[str] = None,
        ros_node: Optional[Node] = None,
        succeed_always: bool = False,
        simulate_tick: bool = False,
    ) -> None:
        super().__init__(
            options=options,
            debug_manager=debug_manager,
            subtree_manager=subtree_manager,
            name=name,
            ros_node=ros_node,
            succeed_always=succeed_always,
            simulate_tick=simulate_tick,
        )

        node_inputs = {}
        node_outputs = {}

        self._action_name = self.options["action_name"]
        self._action_type = self.options["action_type"]

        try:
            self._goal_type = getattr(self.options["action_type"], "Goal")

            if inspect.isclass(self._goal_type):
                msg = self._goal_type()
                for field in msg._fields_and_field_types:
                    node_inputs[field] = type(getattr(msg, field))
            else:
                node_inputs["in"] = self.options["action_type"]
        except AttributeError:
            node_inputs["in"] = self.options["action_type"]
            self.logwarn(f"Non message type passed to: {self.name}")

        try:
            self._result_type = getattr(self.options["action_type"], "Result")
            if inspect.isclass(self._result_type):
                msg = self._result_type()
                for field in msg._fields_and_field_types:
                    node_outputs["result_" + field] = type(getattr(msg, field))
            else:
                node_outputs["result_out"] = self.options["action_type"]
        except AttributeError:
            self._result_type = Int64()
            node_outputs["result_data"] = self.options["action_type"]
            self.logwarn(f"Non message type passed to: {self.name}")

        try:
            self._feedback_type = getattr(self.options["action_type"], "Feedback")
            if inspect.isclass(self._feedback_type):
                msg = self._feedback_type()
                for field in msg._fields_and_field_types:
                    node_outputs["feedback_" + field] = type(getattr(msg, field))
            else:
                node_outputs["feedback_out"] = self.options["action_type"]
        except AttributeError:
            self._feedback_type = Int64()
            node_outputs["feedback_data"] = self.options["action_type"]
            self.logwarn(f"Non message type passed to: {self.name}")

        self._register_node_data(source_map=node_inputs, target_map=self.inputs)
        self._register_node_data(source_map=node_outputs, target_map=self.outputs)

    def _do_setup(self):
        if not self.has_ros_node:
            error_msg = f"Node {self.name} does not have a reference to a ROS node!"
            self.logerr(error_msg)
            raise BehaviorTreeException(error_msg)
        self._lock = Lock()
        self._feedback = None
        self._active_goal = None
        self._result = None

        self._internal_state = ActionStates.IDLE

        self._new_goal_request_future = None
        self._running_goal_handle = None
        self._running_goal_future = None

        self._cancel_goal_future = None

        self._action_available = True
        self._shutdown: bool = False

        self._ac = ActionClient(
            node=self.ros_node,
            action_type=self._action_type,
            action_name=self._action_name,
            callback_group=ReentrantCallbackGroup(),
        )

        if not self._ac.wait_for_server(
            timeout_sec=self.options["wait_for_action_server_seconds"]
        ):
            self._action_available = False
            if (
                "fail_if_not_available" not in self.options
                or not self.options["fail_if_not_available"]
            ):
                raise BehaviorTreeException(
                    f"Action server {self._action_name} not available after waiting "
                    f"{self.options['wait_for_action_server_seconds']} seconds!"
                )

        self._last_goal_time: Optional[Time] = None

        for k, v in self._result_type.get_fields_and_field_types().items():
            self.outputs["result_" + k] = None

        for k, v in self._feedback_type.get_fields_and_field_types().items():
            self.outputs["feedback_" + k] = None

        return NodeMsg.IDLE

    def _feedback_cb(self, feedback) -> None:
        self.logdebug(f"Received feedback message: {feedback}")
        with self._lock:
            self._feedback = feedback

    def _do_tick_wait_for_action_complete(self) -> str:
        if self._running_goal_handle is None or self._running_goal_future is None:
            self._internal_state = ActionStates.FINISHED
            return NodeMsg.BROKEN

        if self._running_goal_future.done():
            self._result = self._running_goal_future.result()
            if self._result is None:
                self._running_goal_handle = None
                self._running_goal_future = None
                self._active_goal = None

                self._internal_state = ActionStates.FINISHED

                self.logdebug("Action result is none, action call must have failed!")
                return NodeMsg.FAILED

            # returns failed except the set.ouput() method returns True
            new_state = NodeMsg.FAILED

            res = self._result.result
            for k, v in res.get_fields_and_field_types().items():
                self.outputs["result_" + k] = getattr(res, k)

            new_state = NodeMsg.SUCCEEDED
            self._running_goal_handle = None
            self._running_goal_future = None
            self._result = None

            self._internal_state = ActionStates.FINISHED

            self.logdebug("Action succeeded, publishing result!")
            return new_state

        if self._running_goal_future.cancelled():
            self._running_goal_handle = None
            self._running_goal_future = None
            self._active_goal = None

            self._internal_state = ActionStates.FINISHED

            self.logwarn("Action execution was cancelled by the remote server!")
            return NodeMsg.FAILED
        seconds_running = (
            self._running_goal_start_time - self.ros_node.get_clock().now()
        ).nanoseconds / 1e9

        if seconds_running > self.options["timeout_seconds"]:
            self.logwarn(f"Cancelling goal after {seconds_running:f} seconds!")

            # This cancels the goal result future, this is not cancelling the goal.
            self._running_goal_future.cancel()
            self._running_goal_future = None

            self._internal_state = ActionStates.REQUEST_GOAL_CANCELLATION
            return NodeMsg.RUNNING
        if self._feedback is not None:
            feed = self._feedback.feedback
            for k, v in feed.get_fields_and_field_types().items():
                self.outputs["feedback_" + k] = getattr(feed, k)

        return NodeMsg.RUNNING

    def _do_tick_cancel_running_goal(self) -> str:
        if self._running_goal_handle is None:
            self.logwarn(
                "Goal cancellation was requested, but there is no handle to the running goal!"
            )
            self._internal_state = ActionStates.FINISHED
            return NodeMsg.BROKEN

        self._cancel_goal_future = self._running_goal_handle.cancel_goal_async()
        self._internal_state = ActionStates.WAITING_FOR_GOAL_CANCELLATION
        return NodeMsg.SUCCEED

    def _do_tick_wait_for_cancel_complete(self) -> str:
        if self._cancel_goal_future is None:
            self.logwarn(
                "Waiting for goal cancellation to complete, but the future is none!"
            )
            self._internal_state = ActionStates.FINISHED
            return NodeMsg.BROKEN

        if self._cancel_goal_future.done():
            self.logdebug("Successfully cancelled goal exectution!")

            self._cancel_goal_future = None
            self._running_goal_handle = None
            self._running_goal_future = None
            self._active_goal = None

            self._internal_state = ActionStates.FINISHED
            return NodeMsg.SUCCEED
        if self._cancel_goal_future.cancelled():
            self.logdebug("Goal cancellation was cancelled!")

            self._cancel_goal_future = None
            self._running_goal_handle = None
            self._running_goal_future = None
            self._active_goal = None

            self._internal_state = ActionStates.FINISHED
            return NodeMsg.FAILURE
        return NodeMsg.RUNNING

    def _do_tick_send_new_goal(self) -> str:
        """Tick to request the execution of a new goal on the action server."""
        self._new_goal_request_future = self._ac.send_goal_async(
            goal=self._input_goal, feedback_callback=self._feedback_cb
        )

        self._active_goal = self._input_goal
        self._internal_state = ActionStates.WAITING_FOR_GOAL_ACCEPTANCE

        return NodeMsg.SUCCEED

    def _do_tick_wait_for_new_goal_complete(self) -> str:
        """Tick to wait for the new goal to be accepted by the action server!."""
        if self._new_goal_request_future is None:
            self.logerr(
                "Waiting for the goal to be accepted"
                "on the action server, but the future is none!"
            )
            self._internal_state = ActionStates.IDLE
            return NodeMsg.BROKEN

        if self._new_goal_request_future.done():
            self._running_goal_handle = self._new_goal_request_future.result()
            self._new_goal_request_future = None

            if self._running_goal_handle is None:
                self.logwarn("Action goal was rejeced by the server!")
                self._internal_state = ActionStates.FINISHED
                return NodeMsg.FAILED

            self._running_goal_start_time = self.ros_node.get_clock().now()
            self._running_goal_future = self._running_goal_handle.get_result_async()

            self._internal_state = ActionStates.WAITING_FOR_ACTION_COMPLETE
            return NodeMsg.SUCCEED

        if self._new_goal_request_future.cancelled():
            self.logwarn("Request for a new goal was cancelled!")
            self._new_goal_request_future = None
            self._running_goal_handle = None
            self._active_goal = None

            self._internal_state = ActionStates.FINISHED
            return NodeMsg.FAILED

        return NodeMsg.RUNNING

    def _do_tick(self):
        if self.simulate_tick:
            self.logdebug("Simulating tick. Action is not executing!")
            if self.succeed_always:
                return NodeMsg.SUCCEEDED

            return NodeMsg.RUNNING

        if not self._action_available:
            if (
                "fail_if_not_available" in self.options
                and self.options["fail_if_not_available"]
            ):
                return NodeMsg.FAILED

        self._input_goal = self._goal_type()
        for k, v in self._input_goal.get_fields_and_field_types().items():
            setattr(self._input_goal, k, self.inputs[k])

        if self._internal_state == ActionStates.IDLE:
            status = self._do_tick_send_new_goal()
            if status not in [NodeMsg.SUCCEED]:
                return status

        if self._internal_state == ActionStates.WAITING_FOR_GOAL_ACCEPTANCE:
            status = self._do_tick_wait_for_new_goal_complete()
            if status not in [NodeMsg.SUCCEED]:
                return status

        if self._internal_state == ActionStates.WAITING_FOR_ACTION_COMPLETE:
            if self._active_goal == self._input_goal:
                return self._do_tick_wait_for_action_complete()
            else:
                # We have a new goal, we should cancel the running one!
                self._internal_state = ActionStates.REQUEST_GOAL_CANCELLATION

        if self._internal_state == ActionStates.REQUEST_GOAL_CANCELLATION:
            status = self._do_tick_cancel_running_goal()
            # Check if goal cancel request was succssful!
            if status not in [NodeMsg.SUCCEED]:
                return status

        if self._internal_state == ActionStates.WAITING_FOR_GOAL_CANCELLATION:
            return self._do_tick_wait_for_cancel_complete()

        if self._internal_state == ActionStates.FINISHED:
            return self._do_tick_finished()

        return NodeMsg.BROKEN

    def _do_tick_finished(self):
        if self._active_goal == self._input_goal:
            return self._state
        else:
            self._internal_state = ActionStates.IDLE
            return NodeMsg.RUNNING

    def _do_untick(self):
        if self._internal_state == ActionStates.WAITING_FOR_ACTION_COMPLETE:
            self._do_tick_cancel_running_goal()

        self._last_goal_time = None
        self._running_goal_future = None
        self._running_goal_handle = None
        self._cancel_goal_future = None
        self._active_goal = None
        self._feedback = None
        self._internal_state = ActionStates.IDLE

        return NodeMsg.IDLE

    def _do_reset(self):
        # same as untick...
        self._do_untick()
        # but also clear the outputs
        for k, v in self._result_type.get_fields_and_field_types().items():
            self.outputs["result_" + k] = None

        for k, v in self._feedback_type.get_fields_and_field_types().items():
            self.outputs["feedback_" + k] = None
        return NodeMsg.IDLE

    def _do_shutdown(self):
        # nothing to do beyond what's done in reset
        self._do_reset()
        self._action_available = False

    def _do_calculate_utility(self):
        if not self.has_ros_node:
            return UtilityBounds(can_execute=False)
        if self._ac is None:
            return UtilityBounds(can_execute=False)
        if not self._ac.server_is_ready():
            return UtilityBounds(can_execute=False)
        return UtilityBounds(
            can_execute=True,
            has_lower_bound_success=True,
            has_upper_bound_success=True,
            has_lower_bound_failure=True,
            has_upper_bound_failure=True,
        )

    """
    def set_action_attributes(self):
        Set all action attributes.
        self._action_type = self.options["action_type"]
        self._goal_type = self.options["goal_type"]
        self._feedback_type = self.options["feedback_type"]
        self._result_type = self.options["result_type"]

        self._action_name = self.options["action_name"]

    def set_goal(self):
        self._input_goal = self.inputs["goal"]

    def set_outputs(self):
        self.outputs["result"] = self._result.result
        return True
    """
