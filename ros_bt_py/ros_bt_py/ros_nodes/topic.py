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

from ros_bt_py.vendor.result import Result, Ok, Err, do

from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from threading import Lock
from rclpy.time import Time

from ros_bt_py_interfaces.msg import UtilityBounds

from ros_bt_py.data_types import (
    BoolType,
    FloatType,
    IntType,
    RosTopicName,
    RosTopicType,
    ReferenceType,
)
from ros_bt_py.exceptions import BehaviorTreeException
from ros_bt_py.helpers import BTNodeState
from ros_bt_py.node import Leaf, define_bt_node
from ros_bt_py.node_config import NodeConfig


@define_bt_node(
    NodeConfig(
        inputs={
            "topic_name": RosTopicName(interface_id=1),
            "topic_type": RosTopicType(interface_id=1),
            "reliable": BoolType(),
            "transient_local": BoolType(),
            "depth": IntType(min_value=0),
        },
        outputs={"message": ReferenceType(reference="topic_type")},
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

    def _do_setup(self) -> Result[BTNodeState, BehaviorTreeException]:
        if not self.has_ros_node:
            error_msg = f"{self.name} does not have a refrence to a ROS node!"
            self.logerr(error_msg)
            return Err(BehaviorTreeException(error_msg))
        self._subscriber = None
        self._msg = None
        return Ok(BTNodeState.IDLE)

    def _callback(self, msg):
        with self._lock:
            self._msg = msg

    def _do_tick(self) -> Result[BTNodeState, BehaviorTreeException]:
        match self.inputs.any_updated(
            "topic_type", "topic_name", "reliable", "transient_local", "depth"
        ):
            case Err(e):
                return Err(e)
            case Ok(b):
                updated = b
        if updated:
            match self._do_reset():
                case Err(e):
                    return Err(e)
                case Ok(_):
                    pass

        if self._subscriber is None:
            match do(
                Ok((r, t, d))
                for r in self.inputs.get_value_as("reliable", bool)
                for t in self.inputs.get_value_as("transient_local", bool)
                for d in self.inputs.get_value_as("depth", int)
            ):
                case Err(e):
                    return Err(e)
                case Ok((r, t, d)):
                    reliable = r
                    transient_local = t
                    depth = d
            reliability_policy = (
                QoSReliabilityPolicy.RELIABLE
                if reliable
                else QoSReliabilityPolicy.BEST_EFFORT
            )
            durability_policy = (
                QoSDurabilityPolicy.TRANSIENT_LOCAL
                if transient_local
                else QoSDurabilityPolicy.VOLATILE
            )
            qos_profile = QoSProfile(
                reliability=reliability_policy,
                durability=durability_policy,
                depth=depth,
            )

            match do(
                Ok((t, n))
                for t in self.inputs.get_value("topic_type")
                for n in self.inputs.get_value_as("topic_name", str)
            ):
                case Err(e):
                    return Err(e)
                case Ok((t, n)):
                    topic_type = t
                    topic_name = n
            self._subscriber = self.ros_node.create_subscription(
                msg_type=topic_type,
                topic=topic_name,
                callback=self._callback,
                qos_profile=qos_profile,
            )

        with self._lock:
            if self._msg is None:
                return Ok(BTNodeState.RUNNING)
            match self.outputs.set_value("message", self._msg):
                case Err(e):
                    return Err(e)
                case Ok(None):
                    self._msg = None
        return Ok(BTNodeState.SUCCEEDED)

    def _do_shutdown(self) -> Result[BTNodeState, BehaviorTreeException]:
        return self._do_reset().map(lambda _: BTNodeState.SHUTDOWN)

    def _do_reset(self) -> Result[BTNodeState, BehaviorTreeException]:
        self._msg = None
        if self._subscriber is None:
            return Ok(BTNodeState.IDLE)
        # Unsubscribe from the topic so we don't receive further updates
        if not self.ros_node.destroy_subscription(self._subscriber):
            error_msg = "Failed to destroy subscription"
            self.logwarn(error_msg)
            return Err(BehaviorTreeException(error_msg))
        self._subscriber = None
        return Ok(BTNodeState.IDLE)

    def _do_untick(self) -> Result[BTNodeState, BehaviorTreeException]:
        return Ok(BTNodeState.IDLE)

    def _do_calculate_utility(self) -> Result[UtilityBounds, BehaviorTreeException]:
        if not self.has_ros_node:
            return Ok(UtilityBounds(can_execute=False))

        match do(
            Ok((r, t, d))
            for r in self.inputs.get_value_as("reliable", bool)
            for t in self.inputs.get_value_as("transient_local", bool)
            for d in self.inputs.get_value_as("depth", int)
        ):
            case Err(e):
                return Err(e)
            case Ok((r, t, d)):
                reliable = r
                transient_local = t
                depth = d
        reliability_policy = (
            QoSReliabilityPolicy.RELIABLE
            if reliable
            else QoSReliabilityPolicy.BEST_EFFORT
        )
        durability_policy = (
            QoSDurabilityPolicy.TRANSIENT_LOCAL
            if transient_local
            else QoSDurabilityPolicy.VOLATILE
        )
        qos_profile = QoSProfile(
            reliability=reliability_policy, durability=durability_policy, depth=depth
        )

        match do(
            Ok((t, n))
            for t in self.inputs.get_value("topic_type")
            for n in self.inputs.get_value_as("topic_name", str)
        ):
            case Err(e):
                return Err(e)
            case Ok((t, n)):
                topic_type = t
                topic_name = n

        resolved_topic = self.ros_node.resolve_topic_name(topic_name)

        for endpoint in self.ros_node.get_publishers_info_by_topic(resolved_topic):
            if (
                endpoint.topic_type == topic_type
                and endpoint.qos_profile == qos_profile
            ):
                # if the topic we want exists, we can do our job, so
                # set all the bounds and leave their values at 0
                return Ok(
                    UtilityBounds(
                        can_execute=True,
                        has_lower_bound_success=True,
                        has_upper_bound_success=True,
                        has_lower_bound_failre=True,
                        has_upper_bound_failure=True,
                    )
                )
        return Ok(UtilityBounds())


@define_bt_node(
    NodeConfig(
        inputs={
            "topic_name": RosTopicName(interface_id=1),
            "topic_type": RosTopicType(interface_id=1),
            "reliable": BoolType(),
            "transient_local": BoolType(),
            "depth": IntType(min_value=0),
            "memory_delay": FloatType(allow_dynamic=False),
        },
        outputs={"message": ReferenceType(reference="topic_type")},
        max_children=0,
    )
)
class TopicMemorySubscriber(Leaf):
    """
    Subscribe to the specified topic and returns FAILED if no message was recently received.

    This node returns RUNNING until it receives its first message.
    When a message is received, it outputs the message and returns SUCCEEDED.
    The message is not cleared until reset is called.
    This node will return FAILED if no message has been received since
    the last memory_delay seconds.
    """

    _lock = Lock()

    def _do_setup(self) -> Result[BTNodeState, BehaviorTreeException]:
        if not self.has_ros_node:
            error_msg = f"{self.name} does not have a refrence to a ROS node!"
            self.logerr(error_msg)
            return Err(BehaviorTreeException(error_msg))
        self._subscriber = None
        self._msg = None
        self._msg_timestamp: Optional[Time] = self.ros_node.get_clock().now()
        match self.inputs.get_value_as("memory_delay", float):
            case Err(e):
                return Err(e)
            case Ok(f):
                self._memory_delay = f
        return Ok(BTNodeState.IDLE)

    def _callback(self, msg):
        with self._lock:
            self._msg = msg
            self._msg_timestamp = self.ros_node.get_clock().now()

    def _do_tick(self) -> Result[BTNodeState, BehaviorTreeException]:
        match self.inputs.any_updated(
            "topic_name", "reliable", "transient_local", "depth"
        ):
            case Err(e):
                return Err(e)
            case Ok(b):
                updated = b
        if updated:
            match self._do_reset():
                case Err(e):
                    return Err(e)
                case Ok(_):
                    pass

        if self._subscriber is None:
            match do(
                Ok((r, t, d))
                for r in self.inputs.get_value_as("reliable", bool)
                for t in self.inputs.get_value_as("transient_local", bool)
                for d in self.inputs.get_value_as("depth", int)
            ):
                case Err(e):
                    return Err(e)
                case Ok((r, t, d)):
                    reliable = r
                    transient_local = t
                    depth = d
            reliability_policy = (
                QoSReliabilityPolicy.RELIABLE
                if reliable
                else QoSReliabilityPolicy.BEST_EFFORT
            )
            durability_policy = (
                QoSDurabilityPolicy.TRANSIENT_LOCAL
                if transient_local
                else QoSDurabilityPolicy.VOLATILE
            )
            qos_profile = QoSProfile(
                reliability=reliability_policy,
                durability=durability_policy,
                depth=depth,
            )

            match do(
                Ok((t, n))
                for t in self.inputs.get_value("topic_type")
                for n in self.inputs.get_value_as("topic_name", str)
            ):
                case Err(e):
                    return Err(e)
                case Ok((t, n)):
                    topic_type = t
                    topic_name = n
            self._subscriber = self.ros_node.create_subscription(
                msg_type=topic_type,
                topic=topic_name,
                callback=self._callback,
                qos_profile=qos_profile,
            )

        with self._lock:
            if self._msg is None:
                return Ok(BTNodeState.RUNNING)
            if self._msg_timestamp is not None:
                if (
                    (self.ros_node.get_clock().now() - self._msg_timestamp).nanoseconds
                    / 1e9
                ) > self._memory_delay:
                    return Ok(BTNodeState.FAILED)
            return self.outputs.set_value("message", self._msg).map(
                lambda _: BTNodeState.SUCCEEDED
            )

    def _do_shutdown(self) -> Result[BTNodeState, BehaviorTreeException]:
        return self._do_reset().map(lambda _: BTNodeState.SHUTDOWN)

    def _do_reset(self) -> Result[BTNodeState, BehaviorTreeException]:
        self._msg = None
        self._msg_timestamp = None
        if self._subscriber is None:
            return Ok(BTNodeState.IDLE)
        # Unsubscribe from the topic so we don't receive further updates
        if not self.ros_node.destroy_subscription(self._subscriber):
            error_msg = "Failed to destroy subscription"
            self.logwarn(error_msg)
            return Err(BehaviorTreeException(error_msg))
        self._subscriber = None
        return Ok(BTNodeState.IDLE)

    def _do_untick(self) -> Result[BTNodeState, BehaviorTreeException]:
        return Ok(BTNodeState.IDLE)

    def _do_calculate_utility(self) -> Result[UtilityBounds, BehaviorTreeException]:
        if not self.has_ros_node:
            return Ok(UtilityBounds(can_execute=False))

        match do(
            Ok((r, t, d))
            for r in self.inputs.get_value_as("reliable", bool)
            for t in self.inputs.get_value_as("transient_local", bool)
            for d in self.inputs.get_value_as("depth", int)
        ):
            case Err(e):
                return Err(e)
            case Ok((r, t, d)):
                reliable = r
                transient_local = t
                depth = d
        reliability_policy = (
            QoSReliabilityPolicy.RELIABLE
            if reliable
            else QoSReliabilityPolicy.BEST_EFFORT
        )
        durability_policy = (
            QoSDurabilityPolicy.TRANSIENT_LOCAL
            if transient_local
            else QoSDurabilityPolicy.VOLATILE
        )
        qos_profile = QoSProfile(
            reliability=reliability_policy, durability=durability_policy, depth=depth
        )

        match do(
            Ok((t, n))
            for t in self.inputs.get_value("topic_type")
            for n in self.inputs.get_value_as("topic_name", str)
        ):
            case Err(e):
                return Err(e)
            case Ok((t, n)):
                topic_type = t
                topic_name = n

        resolved_topic = self.ros_node.resolve_topic_name(topic_name)

        for endpoint in self.ros_node.get_publishers_info_by_topic(resolved_topic):
            if (
                endpoint.topic_type == topic_type
                and endpoint.qos_profile == qos_profile
            ):
                # if the topic we want exists, we can do our job, so
                # set all the bounds and leave their values at 0
                return Ok(
                    UtilityBounds(
                        can_execute=True,
                        has_lower_bound_success=True,
                        has_upper_bound_success=True,
                        has_lower_bound_failre=True,
                        has_upper_bound_failure=True,
                    )
                )
        return Ok(UtilityBounds())


@define_bt_node(
    NodeConfig(
        inputs={
            "topic_name": RosTopicName(interface_id=1),
            "topic_type": RosTopicType(interface_id=1),
            "reliable": BoolType(),
            "transient_local": BoolType(),
            "depth": IntType(min_value=0),
            "message": ReferenceType(reference="topic_type"),
        },
        outputs={},
        max_children=0,
    )
)
class TopicPublisher(Leaf):

    def _do_setup(self) -> Result[BTNodeState, BehaviorTreeException]:
        if not self.has_ros_node:
            error_msg = f"{self.name} does not have a refrence to a ROS node!"
            self.logerr(error_msg)
            return Err(BehaviorTreeException(error_msg))

        self._publisher = None
        return Ok(BTNodeState.IDLE)

    def _do_tick(self) -> Result[BTNodeState, BehaviorTreeException]:
        match self.inputs.any_updated(
            "topic_name", "reliable", "transient_local", "depth"
        ):
            case Err(e):
                return Err(e)
            case Ok(b):
                updated = b
        if updated:
            match self._do_reset():
                case Err(e):
                    return Err(e)
                case Ok(_):
                    pass

        if self._publisher is None:
            match do(
                Ok((r, t, d))
                for r in self.inputs.get_value_as("reliable", bool)
                for t in self.inputs.get_value_as("transient_local", bool)
                for d in self.inputs.get_value_as("depth", int)
            ):
                case Err(e):
                    return Err(e)
                case Ok((r, t, d)):
                    reliable = r
                    transient_local = t
                    depth = d
            reliability_policy = (
                QoSReliabilityPolicy.RELIABLE
                if reliable
                else QoSReliabilityPolicy.BEST_EFFORT
            )
            durability_policy = (
                QoSDurabilityPolicy.TRANSIENT_LOCAL
                if transient_local
                else QoSDurabilityPolicy.VOLATILE
            )
            qos_profile = QoSProfile(
                reliability=reliability_policy,
                durability=durability_policy,
                depth=depth,
            )

            match do(
                Ok((t, n))
                for t in self.inputs.get_value("topic_type")
                for n in self.inputs.get_value_as("topic_name", str)
            ):
                case Err(e):
                    return Err(e)
                case Ok((t, n)):
                    topic_type = t
                    topic_name = n
            self._publisher = self.ros_node.create_publisher(
                msg_type=topic_type,
                topic=topic_name,
                qos_profile=qos_profile,
            )

        # Only publish a new message if our input data has been updated - the
        # old one is latched anyway.
        match self.inputs.any_updated("message"):
            case Err(e):
                return Err(e)
            case Ok(b):
                msg_updated = b
        if msg_updated:
            match self.inputs.get_value("message"):
                case Err(e):
                    return Err(e)
                case Ok(m):
                    message = m
            self._publisher.publish(message)
        return Ok(BTNodeState.SUCCEEDED)

    def _do_shutdown(self) -> Result[BTNodeState, BehaviorTreeException]:
        return self._do_reset().map(lambda _: BTNodeState.SHUTDOWN)

    def _do_reset(self) -> Result[BTNodeState, BehaviorTreeException]:
        if self._publisher is None:
            return Ok(BTNodeState.IDLE)
        if not self.ros_node.destroy_publisher(self._publisher):
            error_msg = "Failed to destroy publisher"
            self.logwarn(error_msg)
            return Err(BehaviorTreeException(error_msg))
        self._publisher = None
        return Ok(BTNodeState.IDLE)

    def _do_untick(self) -> Result[BTNodeState, BehaviorTreeException]:
        return Ok(BTNodeState.IDLE)
