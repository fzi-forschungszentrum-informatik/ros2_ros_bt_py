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
from typing import Optional

from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from threading import Lock
from rclpy.time import Time

from ros_bt_py_interfaces.msg import Node as NodeMsg
from ros_bt_py_interfaces.msg import UtilityBounds

from ros_bt_py.exceptions import BehaviorTreeException
from ros_bt_py.node import Leaf, define_bt_node
from ros_bt_py.node_config import NodeConfig, OptionRef


@define_bt_node(
    NodeConfig(
        version="0.1.0",
        options={
            "topic_type": type,
            "topic_name": str,
            "reliable": bool,
            "transient_local": bool,
            "depth": int,
        },
        inputs={},
        outputs={"message": OptionRef("topic_type")},
        max_children=0,
    )
)
class TopicSubscriber(Leaf):
    """
    Subscribe to the specified topic and output the received messages.

    This node will return RUNNING until a message is received on the topic.
    When a message is received, it outputs the message and returns SUCCEEDED.
    The message is then cleared for the next run.

    This node never returns FAILED.
    """

    _lock = Lock()

    def _do_setup(self):
        self._msg = None

        reliability_policy = (
            QoSReliabilityPolicy.RELIABLE
            if self.options["reliable"]
            else QoSReliabilityPolicy.BEST_EFFORT
        )
        durability_policy = (
            QoSDurabilityPolicy.TRANSIENT_LOCAL
            if self.options["transient_local"]
            else QoSDurabilityPolicy.VOLATILE
        )
        depth = self.options["depth"]

        self._qos_profile = QoSProfile(
            reliability=reliability_policy, history=durability_policy, depth=depth
        )
        self._subscriber = self.ros_node.create_subscription(
            msg_type=self.options["topic_type"],
            topic=self.options["topic_name"],
            callback=self._callback,
            qos_profile=self._qos_profile,
        )
        return NodeMsg.IDLE

    def _callback(self, msg):
        with self._lock:
            self._msg = msg

    def _do_tick(self):
        with self._lock:
            if self._msg is None:
                return NodeMsg.RUNNING
            self.outputs["message"] = self._msg
        return NodeMsg.SUCCEEDED

    def _do_shutdown(self):
        self._msg = None
        if self._subscriber is None:
            return
        # Unsubscribe from the topic so we don't receive further updates
        try:
            self._ros_node.destroy_subscription(self._subscriber)
            self._subscriber = None
        except AttributeError:
            self.logwarn("Can not unregister as no subscriber is available.")

    def _do_reset(self):
        # discard the last received message
        self._msg = None
        return NodeMsg.IDLE

    def _do_untick(self):
        return NodeMsg.IDLE

    def _do_calculate_utility(self):
        if not self.has_ros_node:
            return UtilityBounds(can_execute=False)

        resolved_topic = self.ros_node.resolve_topic_name(self.options["topic_name"])

        for endpoint in self.ros_node.get_publishers_info_by_topic(resolved_topic):
            if (
                endpoint.topic_type == self.options["topic_type"]
                and endpoint.qos_profile == self._qos_profile
            ):
                # if the topic we want exists, we can do our job, so
                # set all the bounds and leave their values at 0
                return UtilityBounds(
                    can_execute=True,
                    has_lower_bound_success=True,
                    has_upper_bound_success=True,
                    has_lower_bound_failre=True,
                    has_upper_bound_failure=True,
                )
        return UtilityBounds()


@define_bt_node(
    NodeConfig(
        version="0.1.0",
        options={
            "topic_type": type,
            "topic_name": str,
            "memory_delay": float,
            "reliable": bool,
            "transient_local": bool,
            "depth": int,
        },
        inputs={},
        outputs={"message": OptionRef("topic_type")},
        max_children=0,
    )
)
class TopicMemorySubscriber(Leaf):
    """
    Subscribe to the specified topic and returns FAILED if no message was recently received.

    This node will return FAILED if no message has been received since
    the last memory_delay seconds.
    When a message is received, it outputs the message and returns SUCCEEDED.
    The message is not cleared for the next runs.

    This node never returns RUNNING.
    """

    _lock = Lock()

    def _do_setup(self):
        if not self.has_ros_node:
            error_msg = f"{self.name} does not have a refrence to a ROS node!"
            self.logerr(error_msg)
            raise BehaviorTreeException(error_msg)

        self._msg = None
        self._msg_timestamp: Optional[Time] = None

        reliability_policy = (
            QoSReliabilityPolicy.RELIABLE
            if self.options["reliable"]
            else QoSReliabilityPolicy.BEST_EFFORT
        )
        durability_policy = (
            QoSDurabilityPolicy.TRANSIENT_LOCAL
            if self.options["transient_local"]
            else QoSDurabilityPolicy.VOLATILE
        )
        depth = self.options["depth"]

        self._qos_profile = QoSProfile(
            reliability=reliability_policy, history=durability_policy, depth=depth
        )
        self._subscriber = self.ros_node.create_subscription(
            msg_type=self.options["topic_type"],
            topic=self.options["topic_name"],
            callback=self._callback,
            qos_profile=self._qos_profile,
        )
        return NodeMsg.IDLE

    def _callback(self, msg):
        with self._lock:
            self._msg = msg
            self._msg_timestamp = self._ros_node.get_clock().now()

    def _do_tick(self):
        with self._lock:
            if self._msg is None:
                if self._msg_timestamp is None:
                    self._msg_timestamp = self.ros_node.get_clock().now()
                    if (
                        (
                            self.ros_node.get_clock().now() - self._msg_timestamp
                        ).nanoseconds
                        / 1e9
                    ) > self.options["memory_delay"]:
                        return NodeMsg.FAILED

                return NodeMsg.RUNNING
            self.outputs["message"] = self._msg
        return NodeMsg.SUCCEEDED

    def _do_shutdown(self):
        self._msg = None
        self._msg_timestamp = None
        if self._subscriber is None:
            return
        # Unsubscribe from the topic so we don't receive further updates
        try:
            self._ros_node.destroy_subscription(self._subscriber)
            self._subscriber = None
        except AttributeError:
            self.logwarn("Can not unregister as no subscriber is available.")

    def _do_reset(self):
        # discard the last received message
        self._msg = None
        self._msg_timestamp = None
        return NodeMsg.IDLE

    def _do_untick(self):
        return NodeMsg.IDLE

    def _do_calculate_utility(self):
        if not self.has_ros_node:
            return UtilityBounds(can_execute=False)

        resolved_topic = self.ros_node.resolve_topic_name(self.options["topic_name"])

        for endpoint in self.ros_node.get_publishers_info_by_topic(resolved_topic):
            if (
                endpoint.topic_type == self.options["topic_type"]
                and endpoint.qos_profile == self._qos_profile
            ):
                # if the topic we want exists, we can do our job, so
                # set all the bounds and leave their values at 0
                return UtilityBounds(
                    can_execute=True,
                    has_lower_bound_success=True,
                    has_upper_bound_success=True,
                    has_lower_bound_failre=True,
                    has_upper_bound_failure=True,
                )
        return UtilityBounds()


@define_bt_node(
    NodeConfig(
        version="1.0.0",
        options={
            "topic_type": type,
            "topic_name": str,
            "reliable": bool,
            "transient_local": bool,
            "depth": int,
        },
        inputs={"message": OptionRef("topic_type")},
        outputs={},
        max_children=0,
    )
)
class TopicPublisher(Leaf):
    def _do_setup(self):
        if not self.has_ros_node:
            error_msg = f"{self.name} does not have a refrence to a ROS node!"
            self.logerr(error_msg)
            raise BehaviorTreeException(error_msg)

        reliability_policy = (
            QoSReliabilityPolicy.RELIABLE
            if self.options["reliable"]
            else QoSReliabilityPolicy.BEST_EFFORT
        )
        durability_policy = (
            QoSDurabilityPolicy.TRANSIENT_LOCAL
            if self.options["transient_local"]
            else QoSDurabilityPolicy.VOLATILE
        )
        depth = self.options["depth"]

        self._qos_profile = QoSProfile(
            reliability=reliability_policy, history=durability_policy, depth=depth
        )

        self._publisher = self.ros_node.create_publisher(
            msg_type=self.options["topic_type"],
            topic=self.options["topic_name"],
            qos_profile=self._qos_profile,
        )
        return NodeMsg.IDLE

    def _do_tick(self):
        # Only publish a new message if our input data has been updated - the
        # old one is latched anyway.
        if self.inputs.is_updated("message"):
            self._publisher.publish(self.inputs["message"])
        return NodeMsg.SUCCEEDED

    def _do_shutdown(self):
        # Unregister the publisher
        try:
            if self._publisher is not None:
                self._ros_node.destroy_publisher(self._publisher)
        except AttributeError:
            self.logwarn("Can not unregister as no publisher is available.")
        self._publisher = None

    def _do_reset(self):
        return NodeMsg.IDLE

    def _do_untick(self):
        return NodeMsg.IDLE
