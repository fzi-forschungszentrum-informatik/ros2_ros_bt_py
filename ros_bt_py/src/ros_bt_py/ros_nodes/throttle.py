# Copyright 2023 FZI Forschungszentrum Informatik

import rclpy

from ros_bt_py_interfaces.msg import Node as NodeMsg

from ros_bt_py.exceptions import BehaviorTreeException
from ros_bt_py.node import Decorator, define_bt_node
from ros_bt_py.node_config import NodeConfig


@define_bt_node(
    NodeConfig(
        version="0.1.0",
        options={"tick_interval": float},
        inputs={},
        outputs={},
        max_children=1,
    )
)
class Throttle(Decorator):
    """
    Wrap a child that stores its success and failures for tick_interval seconds.

    A child that SUCCEEDED or FAILED less than tick_interval seconds will not be ticked.
    This decorator will return the last result until tick_interval seconds elapsed.

    """

    def _do_setup(self):
        if not self.has_ros_node:
            error_msg = f"{self.name} does not have a reference to a ROS node"
            self.logerr(error_msg)
            raise BehaviorTreeException(error_msg)

        self._last_tick = None
        self._last_result = NodeMsg.FAILED
        for child in self.children:
            child.setup()

    def _do_tick(self):
        current_time = self.ros_node.get_clock().now()
        if (
            self._last_tick is None
            or (current_time - self._last_tick).nanoseconds / 1e9
            > self.options["tick_interval"]
        ):
            for child in self.children:
                result = child.tick()
                if result == NodeMsg.RUNNING:
                    return result
                self._last_result = result
                self._last_tick = current_time
                child.reset()
        return self._last_result

    def _do_shutdown(self):
        for child in self.children:
            return child.shutdown()

    def _do_reset(self):
        for child in self.children:
            return child.reset()
        return NodeMsg.IDLE

    def _do_untick(self):
        for child in self.children:
            return child.untick()
        return NodeMsg.IDLE


@define_bt_node(
    NodeConfig(
        version="0.1.0",
        options={"tick_interval": float},
        inputs={},
        outputs={},
        max_children=1,
    )
)
class ThrottleSuccess(Decorator):
    """
    Wrap a child that is prevented to SUCCEEDED multiple times in tick_interval seconds.

    A child that SUCCEEDED less than tick_interval seconds will not be ticked.
    This decorator will return SUCCEEDED once then FAILED until tick_interval seconds elapsed.

    """

    def _do_setup(self):
        if not self.has_ros_node:
            error_msg = f"{self.name} does not have a reference to a ROS node"
            self.logerr(error_msg)
            raise BehaviorTreeException(error_msg)

        self._last_success_tick = None
        for child in self.children:
            child.setup()

    def _do_tick(self):
        current_time = self.ros_node.get_clock().now()
        if (
            self._last_success_tick is None
            or (current_time - self._last_success_tick).nanoseconds / 1e9
            > self.options["tick_interval"]
        ):
            for child in self.children:
                result = child.tick()
                if result == NodeMsg.RUNNING:
                    return result
                if result == NodeMsg.SUCCEEDED:
                    self._last_success_tick = current_time
                    return result
        return NodeMsg.FAILED

    def _do_shutdown(self):
        self._last_success_tick = None
        for child in self.children:
            return child.shutdown()

    def _do_reset(self):
        for child in self.children:
            return child.reset()
        return NodeMsg.IDLE

    def _do_untick(self):
        for child in self.children:
            return child.untick()
        return NodeMsg.IDLE
