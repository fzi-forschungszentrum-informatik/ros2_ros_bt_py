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
from threading import Event, Lock
import abc
import time
from typing import Optional, Any, Dict
from enum import Enum
from ros_bt_py.vendor.result import Result, Ok, Err
import uuid

import rclpy
from rclpy.action.client import ActionClient, ClientGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.time import Time

from ros_bt_py.custom_types import RosActionName, RosActionType
from ros_bt_py_interfaces.msg import UtilityBounds

from ros_bt_py.exceptions import BehaviorTreeException
from ros_bt_py.helpers import BTNodeState
from ros_bt_py.node import Leaf, define_bt_node
from ros_bt_py.node_config import NodeConfig
from rclpy.node import Node
from ros_bt_py.debug_manager import DebugManager
from ros_bt_py.subtree_manager import SubtreeManager
from ros_bt_py.ros_helpers import get_message_field_type


class ActionStates(Enum):
    IDLE = 0
    WAITING_FOR_GOAL_ACCEPTANCE = 1
    WAITING_FOR_ACTION_COMPLETE = 2
    REQUEST_GOAL_CANCELLATION = 3
    WAITING_FOR_GOAL_CANCELLATION = 4
    FINISHED = 5


_SHUTDOWN_CANCEL_TIMEOUT_S = 2.0


def _wait_for_future(future, deadline: float) -> bool:
    if future is None or future.done():
        return True
    completed = Event()
    future.add_done_callback(lambda _: completed.set())
    completed.wait(max(0.0, deadline - time.monotonic()))
    return future.done()


def _cancel_accepted_goal(node, request_future) -> None:
    """Cancel a goal whose acceptance arrived after the node was unticked."""
    if request_future is None or not request_future.done():
        return
    with node._lock:
        if node._shutdown_goal_request_future is not request_future:
            return
        node._shutdown_goal_request_future = None
        try:
            goal_handle = request_future.result()
            if goal_handle is None:
                return
            node._shutdown_goal_handle = goal_handle
            node._shutdown_result_future = goal_handle.get_result_async()
            node._shutdown_cancel_future = goal_handle.cancel_goal_async()
        except Exception as exc:
            node._shutdown_cleanup_error = exc


def _retain_pending_goal_request(node) -> None:
    request_future = node._new_goal_request_future
    node._new_goal_request_future = None
    if request_future is None:
        return
    node._shutdown_goal_request_future = request_future
    request_future.add_done_callback(
        lambda completed: _cancel_accepted_goal(node, completed)
    )


def _retain_running_goal(node) -> Result[None, BehaviorTreeException]:
    if node._running_goal_handle is None:
        return Ok(None)
    node._shutdown_goal_handle = node._running_goal_handle
    node._shutdown_result_future = node._running_goal_future
    if node._shutdown_cancel_future is None:
        try:
            node._shutdown_cancel_future = node._running_goal_handle.cancel_goal_async()
        except Exception as exc:
            return Err(BehaviorTreeException(str(exc)))
    return Ok(None)


def _previous_goal_is_running(node) -> bool:
    request_future = node._shutdown_goal_request_future
    if request_future is not None and request_future.done():
        _cancel_accepted_goal(node, request_future)
    if node._shutdown_cleanup_error is not None:
        # Consume the failed retention instead of pinning the node in RUNNING
        # forever; the tick proceeds with a fresh goal from a clean slate.
        node.logwarn(
            f"Discarding failed goal retention: {node._shutdown_cleanup_error}"
        )
        node._shutdown_cleanup_error = None
        node._shutdown_goal_request_future = None
        node._shutdown_goal_handle = None
        node._shutdown_result_future = None
        node._shutdown_cancel_future = None
        return False
    if (
        node._shutdown_goal_request_future is not None
        and not node._shutdown_goal_request_future.done()
    ):
        return True
    result_future = node._shutdown_result_future
    if result_future is not None and not result_future.done():
        return True
    if result_future is not None:
        node._shutdown_goal_handle = None
        node._shutdown_result_future = None
        node._shutdown_cancel_future = None
    return False


def _cancel_error(cancel_future) -> Optional[str]:
    if cancel_future is None:
        return None
    if cancel_future.cancelled() is True:
        return "Goal cancellation request was cancelled"
    try:
        response = cancel_future.result()
    except Exception as exc:
        return f"Goal cancellation failed: {exc}"
    if hasattr(response, "goals_canceling") and not response.goals_canceling:
        return "Action server rejected goal cancellation"
    return None


def _shutdown_action_client(
    node, ac, timeout_sec
) -> Result[None, BehaviorTreeException]:
    """Wait for remote goal termination without moving the ROS node's executor."""
    deadline = time.monotonic() + timeout_sec
    request_future = node._shutdown_goal_request_future
    if not _wait_for_future(request_future, deadline):
        return Err(BehaviorTreeException("Timed out waiting for goal acceptance"))
    _cancel_accepted_goal(node, request_future)
    cleanup_error = node._shutdown_cleanup_error
    node._shutdown_cleanup_error = None
    if cleanup_error is not None:
        # A stale error must not fail every retry, only the attempt it arose in.
        return Err(BehaviorTreeException(str(cleanup_error)))
    if (
        node._shutdown_cancel_future is None
        and node._shutdown_goal_handle is not None
        and (
            node._shutdown_result_future is None
            or not node._shutdown_result_future.done()
        )
    ):
        try:
            node._shutdown_cancel_future = (
                node._shutdown_goal_handle.cancel_goal_async()
            )
        except Exception as exc:
            return Err(BehaviorTreeException(str(exc)))
    if not _wait_for_future(node._shutdown_cancel_future, deadline):
        return Err(BehaviorTreeException("Timed out waiting for goal cancellation"))
    cancel_error = _cancel_error(node._shutdown_cancel_future)
    if (
        cancel_error is not None
        and node._shutdown_result_future is not None
        and not node._shutdown_result_future.done()
    ):
        node._shutdown_cancel_future = None
        return Err(BehaviorTreeException(cancel_error))
    if not _wait_for_future(node._shutdown_result_future, deadline):
        return Err(BehaviorTreeException("Timed out waiting for action result"))
    if ac is not None:
        ac.destroy()
    node._shutdown_goal_request_future = None
    node._shutdown_goal_handle = None
    node._shutdown_cancel_future = None
    node._shutdown_result_future = None
    return Ok(None)


@define_bt_node(
    NodeConfig(
        options={
            "action_name": RosActionName,
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
class ActionForSetType(Leaf):
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

    _shutdown_cancel_future: Optional[rclpy.Future] = None
    """Most recently issued cancel future, kept alive past _do_untick() clearing
    _cancel_goal_future, so shutdown can wait for it before destroying the client."""

    _action_goal: Optional[Any] = None

    _action_available: bool = True

    _ac: Optional[ActionClient] = None

    @abc.abstractmethod
    def set_action_attributes(self):
        """Set all important action attributes."""
        self._action_type = "ENTER_ACTION_TYPE"
        self._goal_type = "ENTER_GOAL_TYPE"
        self._feedback_type = "ENTER_FEEDBACK_TYPE"
        self._result_type = "ENTER_RESULT_TYPE"

        self._action_name = self.options["action_name"].name

    # TODO What is this supposed to do, should this be flagged as abstract
    def set_input(self):
        pass

    # overwrite, if there is more than one output key to be overwritten
    @abc.abstractmethod
    def set_output_none(self):
        self.outputs["feedback"] = None
        self.outputs["result"] = None

    @abc.abstractmethod
    def set_goal(self):
        self._input_goal = "ENTER_GOAL_FROM_INPUT"

    # Sets the output (in relation to the result) (define output key while overwriting)
    # Should return True, if the node state should be SUCCEEDED after receiving the message
    # and False, if it's in the FAILED state
    @abc.abstractmethod
    def set_outputs(self):
        self.outputs["OUTPUT_KEY"] = self._result  # .result
        return "TRUTHVALUE"

    def _do_setup(self) -> Result[BTNodeState, BehaviorTreeException]:
        if not self.has_ros_node:
            error_msg = f"Node {self.name} does not have a reference to a ROS node!"
            self.logerr(error_msg)
            return Err(BehaviorTreeException(error_msg))
        self._lock = Lock()
        self._feedback = None
        self._active_goal = None
        self._result = None

        self._internal_state = ActionStates.IDLE

        self._new_goal_request_future = None
        self._running_goal_handle = None
        self._running_goal_future = None

        self._cancel_goal_future = None
        self._shutdown_goal_request_future = None
        self._shutdown_goal_handle = None
        self._shutdown_cancel_future = None
        self._shutdown_result_future = None
        self._shutdown_cleanup_error = None
        self._goal_cancel_requested = False

        self._action_available = True
        self._shutdown: bool = False

        self.set_action_attributes()
        # FIXME: ROS Node Optional check not done!
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
                return Err(
                    BehaviorTreeException(
                        f"Action server {self._action_name} not available after waiting "
                        f"{self.options['wait_for_action_server_seconds']} seconds!"
                    )
                )

        self._last_goal_time: Optional[Time] = None
        self.set_output_none()

        return Ok(BTNodeState.IDLE)

    def _feedback_cb(self, feedback) -> None:
        self.logdebug(f"Received feedback message: {feedback}")
        with self._lock:
            self._feedback = feedback

    def _do_tick_wait_for_action_complete(
        self,
    ) -> Result[BTNodeState, BehaviorTreeException]:
        if self._running_goal_handle is None or self._running_goal_future is None:
            self._internal_state = ActionStates.FINISHED
            return Ok(BTNodeState.BROKEN)

        if self._running_goal_future.done():
            self._result = self._running_goal_future.result()
            if self._result is None:
                self._running_goal_handle = None
                self._running_goal_future = None
                self._active_goal = None
                self._goal_cancel_requested = False

                self._internal_state = ActionStates.FINISHED

                self.loginfo("Action result is none, action call must have failed!")
                return Ok(BTNodeState.FAILED)

            # returns failed except the set.ouput() method returns True
            new_state = BTNodeState.FAILED
            if self.set_outputs():
                new_state = BTNodeState.SUCCEEDED
            self._running_goal_handle = None
            self._running_goal_future = None
            self._result = None
            self._goal_cancel_requested = False

            self._internal_state = ActionStates.FINISHED

            self.loginfo("Action succeeded, publishing result!")
            return Ok(new_state)

        if self._running_goal_future.cancelled():
            self._running_goal_handle = None
            self._running_goal_future = None
            self._active_goal = None
            self._goal_cancel_requested = False

            self._internal_state = ActionStates.FINISHED

            self.logwarn("Action execution was cancelled by the remote server!")
            return Ok(BTNodeState.FAILED)
        seconds_running = (
            self.ros_node.get_clock().now() - self._running_goal_start_time
        ).nanoseconds / 1e9

        if (
            seconds_running > self.options["timeout_seconds"]
            and not self._goal_cancel_requested
        ):
            self.logwarn(f"Cancelling goal after {seconds_running:f} seconds!")
            cancel_result = self._do_tick_cancel_running_goal()
            if cancel_result.is_err():
                return cancel_result
            return Ok(BTNodeState.RUNNING)

        return Ok(BTNodeState.RUNNING)

    def _do_tick_cancel_running_goal(
        self,
    ) -> Result[BTNodeState, BehaviorTreeException]:
        if self._running_goal_handle is None:
            self.logwarn(
                "Goal cancellation was requested, but there is no handle to the running goal!"
            )
            self._internal_state = ActionStates.FINISHED
            return Ok(BTNodeState.BROKEN)

        self._cancel_goal_future = self._running_goal_handle.cancel_goal_async()
        # _do_untick() clears _cancel_goal_future right after this; keep a second
        # reference so a subsequent shutdown can still wait for it.
        self._shutdown_cancel_future = self._cancel_goal_future
        self._goal_cancel_requested = True
        self._internal_state = ActionStates.WAITING_FOR_GOAL_CANCELLATION
        return Ok(BTNodeState.SUCCEEDED)

    def _do_tick_wait_for_cancel_complete(
        self,
    ) -> Result[BTNodeState, BehaviorTreeException]:
        if self._cancel_goal_future is None:
            self.logwarn(
                "Waiting for goal cancellation to complete, but the future is none!"
            )
            self._internal_state = ActionStates.FINISHED
            return Ok(BTNodeState.BROKEN)

        if self._cancel_goal_future.done():
            cancel_error = _cancel_error(self._cancel_goal_future)
            if cancel_error is not None:
                # A rejected cancel usually means the goal terminated
                # concurrently; wait for its result instead of erroring.
                self.logwarn(f"Goal cancellation failed: {cancel_error}")
                self._cancel_goal_future = None
                self._shutdown_cancel_future = None
                self._goal_cancel_requested = False
                self._internal_state = ActionStates.WAITING_FOR_ACTION_COMPLETE
                return Ok(BTNodeState.RUNNING)
            self.loginfo("Goal cancellation accepted, waiting for terminal result")
            self._cancel_goal_future = None
            self._internal_state = ActionStates.WAITING_FOR_ACTION_COMPLETE
            return Ok(BTNodeState.RUNNING)
        return Ok(BTNodeState.RUNNING)

    def _do_tick_send_new_goal(self) -> Result[BTNodeState, BehaviorTreeException]:
        """Tick to request the execution of a new goal on the action server."""
        if self._ac is None:
            return Err(BehaviorTreeException("Action client is not initialized"))
        self._new_goal_request_future = self._ac.send_goal_async(
            goal=self._input_goal, feedback_callback=self._feedback_cb
        )

        self._active_goal = self._input_goal
        self._internal_state = ActionStates.WAITING_FOR_GOAL_ACCEPTANCE

        return Ok(BTNodeState.SUCCEEDED)

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
            return Ok(BTNodeState.BROKEN)

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
            return Ok(BTNodeState.SUCCEEDED)

        if self._new_goal_request_future.cancelled():
            self.logwarn("Request for a new goal was cancelled!")
            self._new_goal_request_future = None
            self._running_goal_handle = None
            self._active_goal = None

            self._internal_state = ActionStates.FINISHED
            return Ok(BTNodeState.FAILED)

        return Ok(BTNodeState.RUNNING)

    def _do_tick(self) -> Result[BTNodeState, BehaviorTreeException]:
        if not self._action_available:
            if (
                "fail_if_not_available" in self.options
                and self.options["fail_if_not_available"]
            ):
                return Ok(BTNodeState.FAILED)

        if _previous_goal_is_running(self):
            return Ok(BTNodeState.RUNNING)

        self.set_input()
        self.set_goal()

        if self._internal_state == ActionStates.IDLE:
            status = self._do_tick_send_new_goal()
            if status.ok() not in [BTNodeState.SUCCEEDED]:
                return status

        if self._internal_state == ActionStates.WAITING_FOR_GOAL_ACCEPTANCE:
            status = self._do_tick_wait_for_new_goal_complete()
            if status.ok() not in [BTNodeState.SUCCEEDED]:
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
            if status.ok() not in [BTNodeState.SUCCEEDED]:
                return status

        if self._internal_state == ActionStates.WAITING_FOR_GOAL_CANCELLATION:
            return self._do_tick_wait_for_cancel_complete()

        if self._internal_state == ActionStates.FINISHED:
            return self._do_tick_finished()

        return Ok(BTNodeState.BROKEN)

    def _do_tick_finished(self) -> Result[BTNodeState, BehaviorTreeException]:
        if self._active_goal == self._input_goal:
            return Ok(self._state)
        else:
            self._internal_state = ActionStates.IDLE
            return Ok(BTNodeState.RUNNING)

    def _do_untick(self) -> Result[BTNodeState, BehaviorTreeException]:
        if self._internal_state == ActionStates.WAITING_FOR_GOAL_ACCEPTANCE:
            _retain_pending_goal_request(self)
        elif self._running_goal_handle is not None:
            retain_result = _retain_running_goal(self)
            if retain_result.is_err():
                return retain_result

        self._last_goal_time = None
        self._running_goal_future = None
        self._running_goal_handle = None
        self._cancel_goal_future = None
        self._active_goal = None
        self._feedback = None
        self._internal_state = ActionStates.IDLE

        return Ok(BTNodeState.IDLE)

    def _do_reset(self) -> Result[BTNodeState, BehaviorTreeException]:
        # same as untick...
        untick_result = self._do_untick()
        # but also clear the outputs
        self.outputs["feedback"] = None
        self.outputs["result"] = None
        return untick_result

    def _do_shutdown(self) -> Result[BTNodeState, BehaviorTreeException]:
        reset_result = self._do_reset()
        self._action_available = False
        if reset_result.is_err():
            return reset_result
        shutdown_result = _shutdown_action_client(
            self, self._ac, _SHUTDOWN_CANCEL_TIMEOUT_S
        )
        if shutdown_result.is_err():
            return shutdown_result
        self._ac = None
        return Ok(BTNodeState.SHUTDOWN)

    def _do_calculate_utility(self) -> Result[UtilityBounds, BehaviorTreeException]:
        if not self.has_ros_node:
            return Ok(UtilityBounds(can_execute=False))
        if self._ac is None:
            return Ok(UtilityBounds(can_execute=False))
        if not self._ac.server_is_ready():
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
        version="0.1.0",
        options={
            "action_name": RosActionName,
            "action_type": RosActionType,
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

    _shutdown_cancel_future: Optional[rclpy.Future] = None
    """Most recently issued cancel future, kept alive past _do_untick() clearing
    _cancel_goal_future, so shutdown can wait for it before destroying the client."""

    _internal_state = ActionStates.IDLE

    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)

        node_inputs = {}
        node_outputs = {}

        self._action_name = self.options["action_name"].name
        self._action_type = self.options["action_type"].get_type_obj()

        self._goal_type = self._action_type.Goal
        self._result_type = self._action_type.Result
        self._feedback_type = self._action_type.Feedback

        goal_msg = self._goal_type()
        for field in goal_msg._fields_and_field_types:
            node_inputs[field] = get_message_field_type(goal_msg, field)

        result_msg = self._result_type()
        for field in result_msg._fields_and_field_types:
            node_outputs["result." + field] = get_message_field_type(result_msg, field)

        feedback_msg = self._feedback_type()
        for field in feedback_msg._fields_and_field_types:
            node_outputs["feedback." + field] = get_message_field_type(
                feedback_msg, field
            )

        register_result = self._register_node_data(
            source_map=node_inputs, target_map=self.inputs
        )
        if register_result.is_err():
            raise register_result.unwrap_err()
        register_result = self._register_node_data(
            source_map=node_outputs, target_map=self.outputs
        )
        if register_result.is_err():
            raise register_result.unwrap_err()

    def _do_setup(self) -> Result[BTNodeState, BehaviorTreeException]:
        if not self.has_ros_node:
            error_msg = f"Node {self.name} does not have a reference to a ROS node!"
            self.logerr(error_msg)
            return Err(BehaviorTreeException(error_msg))
        self._lock = Lock()
        self._feedback = None
        self._active_goal = None
        self._result = None

        self._internal_state = ActionStates.IDLE

        self._new_goal_request_future = None
        self._running_goal_handle = None
        self._running_goal_future = None

        self._cancel_goal_future = None
        self._shutdown_goal_request_future = None
        self._shutdown_goal_handle = None
        self._shutdown_cancel_future = None
        self._shutdown_result_future = None
        self._shutdown_cleanup_error = None
        self._goal_cancel_requested = False

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
                or self.options["fail_if_not_available"]
            ):
                return Err(
                    BehaviorTreeException(
                        f"Action server {self._action_name} not available after waiting "
                        f"{self.options['wait_for_action_server_seconds']} seconds!"
                    )
                )

        self._last_goal_time: Optional[Time] = None

        for k, v in self._result_type.get_fields_and_field_types().items():
            self.outputs["result." + k] = None

        for k, v in self._feedback_type.get_fields_and_field_types().items():
            self.outputs["feedback." + k] = None

        return Ok(BTNodeState.IDLE)

    def _feedback_cb(self, feedback) -> None:
        self.logdebug(f"Received feedback message: {feedback}")
        with self._lock:
            self._feedback = feedback

    def _do_tick_wait_for_action_complete(
        self,
    ) -> Result[BTNodeState, BehaviorTreeException]:
        if self._running_goal_handle is None or self._running_goal_future is None:
            self._internal_state = ActionStates.FINISHED
            return Ok(BTNodeState.BROKEN)

        if self._running_goal_future.done():
            self._result = self._running_goal_future.result()
            if self._result is None:
                self._running_goal_handle = None
                self._running_goal_future = None
                self._active_goal = None
                self._goal_cancel_requested = False

                self._internal_state = ActionStates.FINISHED

                self.loginfo("Action result is none, action call must have failed!")
                return Ok(BTNodeState.FAILED)

            # returns failed except the set.ouput() method returns True
            new_state = BTNodeState.FAILED

            res = self._result.result
            for k, v in res.get_fields_and_field_types().items():
                self.outputs["result." + k] = getattr(res, k)

            new_state = BTNodeState.SUCCEEDED
            self._running_goal_handle = None
            self._running_goal_future = None
            self._result = None
            self._goal_cancel_requested = False

            self._internal_state = ActionStates.FINISHED

            self.loginfo("Action succeeded, publishing result!")
            return Ok(new_state)

        if self._running_goal_future.cancelled():
            self._running_goal_handle = None
            self._running_goal_future = None
            self._active_goal = None
            self._goal_cancel_requested = False

            self._internal_state = ActionStates.FINISHED

            self.logwarn("Action execution was cancelled by the remote server!")
            return Ok(BTNodeState.FAILED)
        seconds_running = (
            self.ros_node.get_clock().now() - self._running_goal_start_time
        ).nanoseconds / 1e9

        if (
            seconds_running > self.options["timeout_seconds"]
            and not self._goal_cancel_requested
        ):
            self.logwarn(f"Cancelling goal after {seconds_running:f} seconds!")
            cancel_result = self._do_tick_cancel_running_goal()
            if cancel_result.is_err():
                return cancel_result
            return Ok(BTNodeState.RUNNING)
        if self._feedback is not None:
            feed = self._feedback.feedback
            for k, v in feed.get_fields_and_field_types().items():
                self.outputs["feedback." + k] = getattr(feed, k)

        return Ok(BTNodeState.RUNNING)

    def _do_tick_cancel_running_goal(
        self,
    ) -> Result[BTNodeState, BehaviorTreeException]:
        if self._running_goal_handle is None:
            self.logwarn(
                "Goal cancellation was requested, but there is no handle to the running goal!"
            )
            self._internal_state = ActionStates.FINISHED
            return Ok(BTNodeState.BROKEN)

        self._cancel_goal_future = self._running_goal_handle.cancel_goal_async()
        # _do_untick() clears _cancel_goal_future right after this; keep a second
        # reference so a subsequent shutdown can still wait for it.
        self._shutdown_cancel_future = self._cancel_goal_future
        self._goal_cancel_requested = True
        self._internal_state = ActionStates.WAITING_FOR_GOAL_CANCELLATION
        return Ok(BTNodeState.SUCCEEDED)

    def _do_tick_wait_for_cancel_complete(
        self,
    ) -> Result[BTNodeState, BehaviorTreeException]:
        if self._cancel_goal_future is None:
            self.logwarn(
                "Waiting for goal cancellation to complete, but the future is none!"
            )
            self._internal_state = ActionStates.FINISHED
            return Ok(BTNodeState.BROKEN)

        if self._cancel_goal_future.done():
            cancel_error = _cancel_error(self._cancel_goal_future)
            if cancel_error is not None:
                # A rejected cancel usually means the goal terminated
                # concurrently; wait for its result instead of erroring.
                self.logwarn(f"Goal cancellation failed: {cancel_error}")
                self._cancel_goal_future = None
                self._shutdown_cancel_future = None
                self._goal_cancel_requested = False
                self._internal_state = ActionStates.WAITING_FOR_ACTION_COMPLETE
                return Ok(BTNodeState.RUNNING)
            self.loginfo("Goal cancellation accepted, waiting for terminal result")
            self._cancel_goal_future = None
            self._internal_state = ActionStates.WAITING_FOR_ACTION_COMPLETE
            return Ok(BTNodeState.RUNNING)
        return Ok(BTNodeState.RUNNING)

    def _do_tick_send_new_goal(self) -> Result[BTNodeState, BehaviorTreeException]:
        """Tick to request the execution of a new goal on the action server."""
        if self._ac is None:
            return Err(BehaviorTreeException("Action client is not initialized"))
        self._new_goal_request_future = self._ac.send_goal_async(
            goal=self._input_goal, feedback_callback=self._feedback_cb
        )

        self._active_goal = self._input_goal
        self._internal_state = ActionStates.WAITING_FOR_GOAL_ACCEPTANCE

        return Ok(BTNodeState.SUCCEEDED)

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
            return Ok(BTNodeState.BROKEN)

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
            return Ok(BTNodeState.SUCCEEDED)

        if self._new_goal_request_future.cancelled():
            self.logwarn("Request for a new goal was cancelled!")
            self._new_goal_request_future = None
            self._running_goal_handle = None
            self._active_goal = None

            self._internal_state = ActionStates.FINISHED
            return Ok(BTNodeState.FAILED)

        return Ok(BTNodeState.RUNNING)

    def _do_tick(self) -> Result[BTNodeState, BehaviorTreeException]:
        if not self._action_available:
            if (
                "fail_if_not_available" in self.options
                and self.options["fail_if_not_available"]
            ):
                return Ok(BTNodeState.FAILED)

        if _previous_goal_is_running(self):
            return Ok(BTNodeState.RUNNING)

        self._input_goal = self._goal_type()
        for k, v in self._input_goal.get_fields_and_field_types().items():
            setattr(self._input_goal, k, self.inputs[k])

        if self._internal_state == ActionStates.IDLE:
            status = self._do_tick_send_new_goal()
            if status.ok() not in [BTNodeState.SUCCEEDED]:
                return status

        if self._internal_state == ActionStates.WAITING_FOR_GOAL_ACCEPTANCE:
            status = self._do_tick_wait_for_new_goal_complete()
            if status.ok() not in [BTNodeState.SUCCEEDED]:
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
            if status.ok() not in [BTNodeState.SUCCEEDED]:
                return status

        if self._internal_state == ActionStates.WAITING_FOR_GOAL_CANCELLATION:
            return self._do_tick_wait_for_cancel_complete()

        if self._internal_state == ActionStates.FINISHED:
            return self._do_tick_finished()

        return Ok(BTNodeState.BROKEN)

    def _do_tick_finished(self) -> Result[BTNodeState, BehaviorTreeException]:
        if self._active_goal == self._input_goal:
            return Ok(self._state)
        else:
            self._internal_state = ActionStates.IDLE
            return Ok(BTNodeState.RUNNING)

    def _do_untick(self) -> Result[BTNodeState, BehaviorTreeException]:
        if self._internal_state == ActionStates.WAITING_FOR_GOAL_ACCEPTANCE:
            _retain_pending_goal_request(self)
        elif self._running_goal_handle is not None:
            retain_result = _retain_running_goal(self)
            if retain_result.is_err():
                return retain_result

        self._last_goal_time = None
        self._running_goal_future = None
        self._running_goal_handle = None
        self._cancel_goal_future = None
        self._active_goal = None
        self._feedback = None
        self._internal_state = ActionStates.IDLE

        return Ok(BTNodeState.IDLE)

    def _do_reset(self) -> Result[BTNodeState, BehaviorTreeException]:
        # same as untick...
        untick_result = self._do_untick()
        # but also clear the outputs
        for k, v in self._result_type.get_fields_and_field_types().items():
            self.outputs["result." + k] = None

        for k, v in self._feedback_type.get_fields_and_field_types().items():
            self.outputs["feedback." + k] = None
        return untick_result

    def _do_shutdown(self) -> Result[BTNodeState, BehaviorTreeException]:
        reset_result = self._do_reset()
        self._action_available = False
        if reset_result.is_err():
            return reset_result
        shutdown_result = _shutdown_action_client(
            self, self._ac, _SHUTDOWN_CANCEL_TIMEOUT_S
        )
        if shutdown_result.is_err():
            return shutdown_result
        self._ac = None
        return Ok(BTNodeState.SHUTDOWN)

    def _do_calculate_utility(self) -> Result[UtilityBounds, BehaviorTreeException]:
        if not self.has_ros_node:
            return Ok(UtilityBounds(can_execute=False))
        if self._ac is None:
            return Ok(UtilityBounds(can_execute=False))
        if not self._ac.server_is_ready():
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
