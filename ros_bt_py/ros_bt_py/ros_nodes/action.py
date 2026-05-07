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
import abc
from typing import Optional, Any
from enum import Enum

from ros_bt_py.vendor.result import Result, Ok, Err, do

import rclpy
from rclpy.action.client import ActionClient, ClientGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.time import Time

from ros_bt_py.data_types import (
    BoolType,
    DataContainer,
    FloatType,
    RosActionName,
    RosActionType,
    get_message_field_io_type,
)
from ros_bt_py.exceptions import BehaviorTreeException, NodeConfigError
from ros_bt_py.helpers import BTNodeState
from ros_bt_py.node import Leaf, define_bt_node
from ros_bt_py.node_config import NodeConfig

from ros_bt_py_interfaces.msg import UtilityBounds


class ActionStates(Enum):
    IDLE = 0
    WAITING_FOR_GOAL_ACCEPTANCE = 1
    WAITING_FOR_ACTION_COMPLETE = 2
    REQUEST_GOAL_CANCELLATION = 3
    WAITING_FOR_GOAL_CANCELLATION = 4
    FINISHED = 5


@define_bt_node(
    NodeConfig(
        inputs={
            "action_name": RosActionName(interface_id=1),
            "timeout_seconds": FloatType(allow_dynamic=False, value=30.2),
        },
        outputs={},
        max_children=0,
    )
)
class ActionBase(Leaf):
    """
    Connect to a ROS action and sends the supplied goal.

    Will always return RUNNING on the tick a new goal is sent, even if
    the server replies really quickly!

    On every tick, outputs['feedback'] and outputs['result'] (if
    available) are updated.

    On untick, reset or shutdown, the goal is cancelled and will be
    re-sent on the next tick.
    """

    _action_type: type
    _action_client: Optional[ActionClient] = None
    _feedback = None
    _internal_state: ActionStates

    # Returns the action goal message that should be send to the server.
    @abc.abstractmethod
    def get_goal(self) -> Result[Any, BehaviorTreeException]:
        raise NotImplementedError

    # Property for the list of goal fields that have to be monitored for updates
    @property
    @abc.abstractmethod
    def goal_fields(self) -> list[str]:
        raise NotImplementedError

    # Sets the outputs relative to the given result.
    # Should return the desired node state as `Ok(...)` or an `Err(...)` with an exception.
    @abc.abstractmethod
    def set_result_outputs(
        self, result: Any
    ) -> Result[BTNodeState, BehaviorTreeException]:
        raise NotImplementedError

    # Sets the outputs relative to the given feedback.
    # Should return `Ok(None)` or an `Err(...)` with an exception.
    # Feedback doesn't affect the node state
    @abc.abstractmethod
    def set_feedback_outputs(
        self, feedback: Any
    ) -> Result[None, BehaviorTreeException]:
        raise NotImplementedError

    # Get the action type
    @abc.abstractmethod
    def get_action_type(self) -> Result[type, NodeConfigError]:
        raise NotImplementedError

    def _do_setup(self) -> Result[BTNodeState, BehaviorTreeException]:
        if not self.has_ros_node:
            error_msg = f"Node {self.name} does not have a reference to a ROS node!"
            self.logerr(error_msg)
            return Err(BehaviorTreeException(error_msg))

        match self.inputs.get_value_as("timeout_seconds", float):
            case Err(e):
                return Err(e)
            case Ok(f):
                self._timeout_seconds = f

        match self.get_action_type():
            case Err(e):
                return Err(e)
            case Ok(t):
                self._action_type = t

        self._lock = Lock()
        self._feedback = None

        self._internal_state = ActionStates.IDLE

        self._new_goal_request_future = None
        self._running_goal_handle = None
        self._running_goal_future = None
        self._cancel_goal_future = None

        self._action_client = None

        self._last_goal_time: Optional[Time] = None

        return Ok(BTNodeState.IDLE)

    def _feedback_cb(self, feedback) -> None:
        self.logdebug(f"Received feedback message: {feedback}")
        with self._lock:
            self._feedback = feedback

    def _do_tick_wait_for_action_complete(
        self,
    ) -> Result[BTNodeState, BehaviorTreeException]:
        if self._running_goal_handle is None or self._running_goal_future is None:
            return Err(BehaviorTreeException("Current running goal is unavailable"))

        if self._running_goal_future.done():
            result = self._running_goal_future.result()
            if result is None:
                self._running_goal_handle = None
                self._running_goal_future = None
                self._internal_state = ActionStates.FINISHED
                self.loginfo("Action result is none, action call must have failed!")
                return Ok(BTNodeState.FAILED)

            self._running_goal_handle = None
            self._running_goal_future = None
            self._internal_state = ActionStates.FINISHED
            self.loginfo("Action succeeded, publishing result!")
            return self.set_result_outputs(result.result)

        if self._running_goal_future.cancelled():
            self._running_goal_handle = None
            self._running_goal_future = None
            self._internal_state = ActionStates.FINISHED
            self.logwarn("Action execution was cancelled by the remote server!")
            return Ok(BTNodeState.FAILED)

        seconds_running = (
            self.ros_node.get_clock().now() - self._running_goal_start_time
        ).nanoseconds / 1e9

        if seconds_running > self._timeout_seconds:
            self.logwarn(f"Cancelling goal after {seconds_running:f} seconds!")

            # This cancels the goal result future, this is not cancelling the goal.
            self._running_goal_future.cancel()
            self._running_goal_future = None

            self._internal_state = ActionStates.REQUEST_GOAL_CANCELLATION
            return Ok(BTNodeState.RUNNING)

        if self._feedback is not None:
            with self._lock:
                self.set_feedback_outputs(self._feedback.feedback)
                self._feedback = None

        return Ok(BTNodeState.RUNNING)

    def _do_tick_cancel_running_goal(
        self,
    ) -> Result[BTNodeState, BehaviorTreeException]:
        if self._running_goal_handle is None:
            self.logwarn(
                "Goal cancellation was requested, but there is no handle to the running goal!"
            )
            return Err(BehaviorTreeException("Running goal handle is not available"))

        self._cancel_goal_future = self._running_goal_handle.cancel_goal_async()
        self._internal_state = ActionStates.WAITING_FOR_GOAL_CANCELLATION
        return Ok(BTNodeState.RUNNING)

    def _do_tick_wait_for_cancel_complete(
        self,
    ) -> Result[BTNodeState, BehaviorTreeException]:
        if self._cancel_goal_future is None:
            self.logwarn(
                "Waiting for goal cancellation to complete, but the future is none!"
            )
            self._internal_state = ActionStates.FINISHED
            return Err(BehaviorTreeException("Cancel goal future is not available"))

        if self._cancel_goal_future.done():
            self.loginfo("Successfully cancelled goal exectution!")

            if self._input_goal is not None:
                state = BTNodeState.RUNNING
            else:
                state = BTNodeState.FAILED

            self._cancel_goal_future = None
            self._running_goal_handle = None
            self._running_goal_future = None

            self._internal_state = ActionStates.FINISHED
            return Ok(state)
        if self._cancel_goal_future.cancelled():
            self.logdebug("Goal cancellation was cancelled!")

            self._cancel_goal_future = None
            self._running_goal_handle = None
            self._running_goal_future = None

            self._internal_state = ActionStates.FINISHED
            return Ok(BTNodeState.FAILED)
        return Ok(BTNodeState.RUNNING)

    def _do_tick_send_new_goal(self) -> Result[BTNodeState, BehaviorTreeException]:
        """Tick to request the execution of a new goal on the action server."""
        if self._action_client is None:
            return Err(BehaviorTreeException("No action client available"))
        self._new_goal_request_future = self._action_client.send_goal_async(
            goal=self._input_goal, feedback_callback=self._feedback_cb
        )

        self._input_goal = None
        self._internal_state = ActionStates.WAITING_FOR_GOAL_ACCEPTANCE

        return Ok(BTNodeState.RUNNING)

    def _do_tick_wait_for_new_goal_complete(
        self,
    ) -> Result[BTNodeState, BehaviorTreeException]:
        """Tick to wait for the new goal to be accepted by the action server!."""
        if self._new_goal_request_future is None:
            self.logerr(
                "Waiting for the goal to be accepted"
                "on the action server, but the future is none!"
            )
            self._internal_state = ActionStates.IDLE
            return Err(BehaviorTreeException("No goal request future"))

        if self._new_goal_request_future.done():
            self._running_goal_handle = self._new_goal_request_future.result()
            self._new_goal_request_future = None

            if self._running_goal_handle is None:
                self.logwarn("Action goal was rejeced by the server!")
                self._internal_state = ActionStates.FINISHED
                return Ok(BTNodeState.FAILED)

            self._running_goal_start_time = self.ros_node.get_clock().now()
            self._running_goal_future = self._running_goal_handle.get_result_async()

            self._internal_state = ActionStates.WAITING_FOR_ACTION_COMPLETE
            return Ok(BTNodeState.RUNNING)

        if self._new_goal_request_future.cancelled():
            self.logwarn("Request for a new goal was cancelled!")
            self._new_goal_request_future = None
            self._running_goal_handle = None

            self._internal_state = ActionStates.FINISHED
            return Ok(BTNodeState.FAILED)

        return Ok(BTNodeState.RUNNING)

    def _do_tick(self) -> Result[BTNodeState, BehaviorTreeException]:
        match self.inputs.any_updated("action_name", *self.goal_fields):
            case Err(e):
                return Err(e)
            case Ok(b):
                updated = b
        # If the current state is idle, this is the first tick after setup, reset or untick
        if updated:
            match self.get_goal():
                case Err(e):
                    return Err(e)
                case Ok(g):
                    self._input_goal = g
        if updated and self._internal_state not in [
            ActionStates.IDLE,
            ActionStates.FINISHED,
            ActionStates.WAITING_FOR_GOAL_CANCELLATION,
        ]:
            self._internal_state = ActionStates.REQUEST_GOAL_CANCELLATION

        if self._action_client is None:
            match self.inputs.get_value_as("action_name", str):
                case Err(e):
                    return Err(e)
                case Ok(s):
                    action_name = s
            self._action_client = ActionClient(
                node=self.ros_node,
                action_type=self._action_type,
                action_name=action_name,
                callback_group=ReentrantCallbackGroup(),
            )

        if self._internal_state == ActionStates.IDLE:
            return self._do_tick_send_new_goal()

        if self._internal_state == ActionStates.WAITING_FOR_GOAL_ACCEPTANCE:
            return self._do_tick_wait_for_new_goal_complete()

        if self._internal_state == ActionStates.WAITING_FOR_ACTION_COMPLETE:
            return self._do_tick_wait_for_action_complete()

        if self._internal_state == ActionStates.REQUEST_GOAL_CANCELLATION:
            return self._do_tick_cancel_running_goal()

        if self._internal_state == ActionStates.WAITING_FOR_GOAL_CANCELLATION:
            return self._do_tick_wait_for_cancel_complete()

        if self._internal_state == ActionStates.FINISHED:
            return self._do_tick_finished()

        return Err(
            BehaviorTreeException(f"Internal state is broken: {self._internal_state}")
        )

    def _do_tick_finished(self) -> Result[BTNodeState, BehaviorTreeException]:
        if self._input_goal is None:
            return Ok(self.state)
        else:
            if self._action_client is not None:
                self._action_client.destroy()
                self._action_client = None

            self._internal_state = ActionStates.IDLE
            return Ok(BTNodeState.RUNNING)

    def _do_untick(self) -> Result[BTNodeState, BehaviorTreeException]:
        if self._internal_state == ActionStates.WAITING_FOR_ACTION_COMPLETE:
            cancel_result = self._do_tick_cancel_running_goal()
            if cancel_result.is_err():
                return cancel_result

        self._last_goal_time = None
        self._running_goal_future = None
        self._running_goal_handle = None
        self._cancel_goal_future = None
        self._feedback = None
        self._internal_state = ActionStates.IDLE

        return Ok(BTNodeState.IDLE)

    def _do_reset(self) -> Result[BTNodeState, BehaviorTreeException]:
        match self._do_untick():
            case Err(e):
                return Err(e)
            case Ok(_):
                pass
        if self._action_client is not None:
            self._action_client.destroy()
            self._action_client = None
        return Ok(BTNodeState.IDLE)

    def _do_shutdown(self) -> Result[BTNodeState, BehaviorTreeException]:
        # nothing to do beyond what's done in reset
        reset_result = self._do_reset()
        if reset_result.is_err():
            return reset_result
        return Ok(BTNodeState.SHUTDOWN)

    def _do_calculate_utility(self) -> Result[UtilityBounds, BehaviorTreeException]:
        if not self.has_ros_node:
            return Ok(UtilityBounds(can_execute=False))
        if self._action_client is None:
            return Ok(UtilityBounds(can_execute=False))
        if not self._action_client.server_is_ready():
            return Ok(UtilityBounds(can_execute=False))
        return Ok(
            UtilityBounds(
                can_execute=True,
                has_lower_bound_success=True,
                has_upper_bound_success=True,
                has_lower_bound_failure=True,
                has_upper_bound_failure=True,
            )
        )


@define_bt_node(
    NodeConfig(
        inputs={"action_type": RosActionType(interface_id=1)},
        outputs={},
        max_children=0,
    )
)
class Action(ActionBase):

    def get_goal(self):
        goal = self._goal_type()
        for key in self._goal_fields:
            match self.inputs.get_value(key):
                case Err(e):
                    return Err(e)
                case Ok(v):
                    setattr(goal, key, v)
        return Ok(goal)

    @property
    def goal_fields(self):
        return self._goal_fields

    def set_result_outputs(self, result):
        try:
            result_values = {
                f"result.{key}": getattr(result, key) for key in self._result_fields
            }
        except AttributeError as e:
            return Err(BehaviorTreeException(str(e)))
        return self.outputs.set_multiple_values(**result_values).map(
            lambda _: BTNodeState.SUCCEEDED
        )

    def set_feedback_outputs(self, feedback):
        try:
            feedback_values = {
                f"feedback.{key}": getattr(feedback, key)
                for key in self._feedback_fields
            }
        except AttributeError as e:
            return Err(BehaviorTreeException(str(e)))
        return self.outputs.set_multiple_values(**feedback_values)

    def get_action_type(self):
        return self.inputs.get_value_as("action_type", type)

    def add_extra_inputs(self) -> Result[dict[str, DataContainer], NodeConfigError]:
        match self.get_action_type():
            case Err(e):
                return Err(e)
            case Ok(t):
                action_type = t
        self._goal_type = action_type.Goal
        inputs = {}
        for (
            field_name,
            field_type,
        ) in self._goal_type.get_fields_and_field_types().items():
            match get_message_field_io_type(field_type):
                case Err(e):
                    return Err(NodeConfigError(e))
                case Ok(t):
                    inputs[field_name] = t
        return Ok(inputs)

    def add_extra_outputs(self) -> Result[dict[str, DataContainer], NodeConfigError]:
        match self.get_action_type():
            case Err(e):
                return Err(e)
            case Ok(t):
                action_type = t
        self._result_type = action_type.Result
        self._feedback_type = action_type.Feedback
        combined_fields: dict[str, str] = {}
        for (
            field_name,
            field_type,
        ) in self._result_type.get_fields_and_field_types().items():
            combined_fields[f"result.{field_name}"] = field_type
        for (
            field_name,
            field_type,
        ) in self._result_type.get_fields_and_field_types().items():
            combined_fields[f"feedback.{field_name}"] = field_type
        outputs = {}
        for field_name, field_type in combined_fields.items():
            match get_message_field_io_type(field_type):
                case Err(e):
                    return Err(NodeConfigError(e))
                case Ok(t):
                    outputs[field_name] = t
        return Ok(outputs)

    def _do_setup(self):
        self._goal_fields = list(self._goal_type.get_fields_and_field_types().keys())
        self._result_fields = list(
            self._result_type.get_fields_and_field_types().keys()
        )
        self._feedback_fields = list(
            self._feedback_type.get_fields_and_field_types().keys()
        )
        return super()._do_setup()
