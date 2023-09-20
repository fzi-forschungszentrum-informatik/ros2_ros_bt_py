# Copyright 2018-2023 FZI Forschungszentrum Informatik

from threading import Lock
from abc import ABC, abstractmethod
from typing import Optional

import rclpy
from rclpy.action.client import ActionClient, ClientGoalHandle
from rclpy.node import ReentrantCallbackGroup
from rclpy.time import Time
from action_msgs.msg import GoalStatus

from ros_bt_py_interfaces.msg import Node as NodeMsg
from ros_bt_py_interfaces.msg import UtilityBounds

from ros_bt_py.exceptions import BehaviorTreeException
from ros_bt_py.node import Leaf, define_bt_node
from ros_bt_py.node_config import NodeConfig, OptionRef


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

    """

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

    def _do_setup(self):
        if not self.has_ros_node:
            error_msg = f"Node {self.name} does not have a reference to a ROS node!"
            self.logerr(error_msg)
            raise BehaviorTreeException(error_msg)
        self._lock = Lock()
        self._feedback = None

        self._new_goal_future: Optional[rclpy.Future] = None
        self._running_goal_handle: Optional[ClientGoalHandle] = None
        self._running_goal_future: Optional[rclpy.Future] = None

        self._cancel_goal_future: Optional[rclpy.Future] = None

        self._action_available: bool = True
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
        with self._lock:
            self._feedback = feedback

    def _do_tick_wait_for_action_complete(self) -> str:
        if self._running_goal_handle is None or self._running_goal_future is None:
            return NodeMsg.BROKEN
        if self._running_goal_future.done():
            result = self._running_goal_future.result()
            if result is None:
                self._running_goal_handle = None
                self._running_goal_future = None
                self._active_goal = None
                return NodeMsg.FAILED
            self.outputs["result"] = result
            self._running_goal_handle = None
            self._running_goal_future = None
            self._active_goal = None
            return NodeMsg.SUCCEED
        if self._running_goal_future.cancelled():
            self._running_goal_handle = None
            self._running_goal_future = None
            self._active_goal = None
            return NodeMsg.FAILED
        seconds_running = (
            self._running_goal_start_time - self.ros_node.get_clock().now()
        ).nanoseconds / 1e9
        if seconds_running > self.options["timeout_seconds"]:
            self.logwarn(f"Cancelling goal after {seconds_running:f} seconds!")
            self._running_goal_future.cancel()
            self._running_goal_future = None
            status = self._do_tick_cancel_running_goal()
            if status != NodeMsg.SUCCEED:
                return status
        return NodeMsg.RUNNING

    def _do_tick_cancel_running_goal(self) -> str:
        if self._running_goal_handle is None or self._cancel_goal_future is not None:
            return NodeMsg.BROKEN
        self._cancel_goal_future = self._running_goal_handle.cancel_goal_async()
        return NodeMsg.RUNNING

    def _do_tick_wait_for_cancel_complete(self) -> str:
        if self._cancel_goal_future is None:
            return NodeMsg.BROKEN
        if self._cancel_goal_future.done():
            self._cancel_goal_future = None
            self._running_goal_handle = None
            self._running_goal_future = None
            self._active_goal = None
            return NodeMsg.SUCCEED
        if self._cancel_goal_future.cancelled():
            self._cancel_goal_future = None
            self._running_goal_handle = None
            self._running_goal_future = None
            self._active_goal = None
            return NodeMsg.FAIL
        return NodeMsg.RUNNING

    def _do_tick_send_new_goal(self) -> str:
        if self._running_goal_handle is not None or self._new_goal_future is not None:
            return NodeMsg.BROKEN
        self._new_goal_future = self._ac.send_goal_async(
            goal=self._input_goal, feedback_callback=self._feedback_cb
        )
        self._active_goal = self._input_goal
        return NodeMsg.SUCCEED

    def _do_tick_wait_for_new_goal_complete(self) -> str:
        if self._new_goal_future is None:
            return NodeMsg.BROKEN
        if self._new_goal_future.done():
            self._running_goal_handle: Optional[
                ClientGoalHandle
            ] = self._new_goal_future.result()
            self._new_goal_future = None
            self._active_goal = None
            if self._running_goal_handle is None:
                self.logerr(
                    "Could not create new action goal, future returned failure!"
                )
                return NodeMsg.FAILED
            self._running_goal_start_time = self.ros_node.get_clock().now()
            self._running_goal_future = self._running_goal_handle.get_result_async()
            return NodeMsg.SUCCEED
        if self._new_goal_future.cancelled():
            self.logerr("Request for a new goal was cancelled!")
            self._new_goal_future = None
            self._running_goal_handle = None
            self._active_goal = None
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

        # Check if we have an active goal
        if (
            self._running_goal_handle is not None
            and self._running_goal_handle.status == GoalStatus.STATUS_EXECUTING
        ):
            # Check if goal changed
            if self._active_goal == self._input_goal:
                return self._do_tick_wait_for_action_complete()
            else:
                status = self._do_tick_cancel_running_goal()
                if status not in [NodeMsg.SUCCEED]:
                    return status
        # Check if we are waiting for the cancellation of a goal
        if self._cancel_goal_future is not None:
            status = self._do_tick_wait_for_cancel_complete()
            if status in [NodeMsg.FAILED, NodeMsg.RUNNING]:
                return status
        # Create a new goal as we do not have one!
        if self._do_tick_send_new_goal() == NodeMsg.FAILED:
            return NodeMsg.FAILED

        # Check if we are waiting for the creation of a new request
        if self._new_goal_future is not None:
            status = self._do_tick_wait_for_new_goal_complete()
            if status in [NodeMsg.FAILED, NodeMsg.RUNNING]:
                return status
            else:
                return NodeMsg.RUNNING
        return NodeMsg.BROKEN

    def _do_untick(self):
        if self._active_goal is not None or self._running_goal_future is not None:
            self._do_tick_cancel_running_goal()
        self._last_goal_time = None
        self._running_goal_future = None
        self._running_goal_handle = None
        self._cancel_goal_future = None
        self._active_goal = None
        self._feedback = None
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
            "goal_type": type,
            "feedback_type": type,
            "result_type": type,
        },
        inputs={"goal": OptionRef("goal_type")},
        outputs={
            "feedback": OptionRef("feedback_type"),
            "result": OptionRef("result_type"),
        },
        max_children=0,
    )
)
class Action(ActionForSetType):
    """
    Connect to a ROS action and sends the supplied goal.

    Will always return RUNNING on the tick a new goal is sent, even if
    the server replies really quickly!

    On every tick, outputs['feedback'] and outputs['result'] (if
    available) are updated.

    On untick, reset or shutdown, the goal is cancelled and will be
    re-sent on the next tick.
    """

    def set_action_attributes(self):
        """Set all action attributes."""
        self._action_type = self.options["action_type"]
        self._goal_type = self.options["goal_type"]
        self._feedback_type = self.options["feedback_type"]
        self._result_type = self.options["result_type"]

        self._action_name = self.options["action_name"]

    def set_goal(self):
        self._input_goal = self.inputs["goal"]
