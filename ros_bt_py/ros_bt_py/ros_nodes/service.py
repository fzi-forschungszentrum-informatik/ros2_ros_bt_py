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
from rclpy.task import Future
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.client import Client
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time

from ros_bt_py.custom_types import RosServiceName, RosServiceType
from ros_bt_py_interfaces.msg import Node as NodeMsg
from ros_bt_py_interfaces.msg import UtilityBounds

from ros_bt_py.debug_manager import DebugManager
from ros_bt_py.subtree_manager import SubtreeManager
from ros_bt_py.node import Leaf, define_bt_node
from ros_bt_py.node_config import NodeConfig, OptionRef
from ros_bt_py.exceptions import BehaviorTreeException

from abc import ABC, abstractmethod
from typing import Any, Optional, Dict

import inspect


@define_bt_node(
    NodeConfig(
        version="0.1.0",
        options={
            "service_type": RosServiceType,
            "wait_for_response_seconds": float,
        },
        inputs={
            "service_name": str,
        },
        outputs={},
        max_children=0,
    )
)
class ServiceInput(Leaf):
    """
    Call a ROS service with the provided Request data.

    To make sure the service call cannot block the :meth:`tick()`
    method, this uses a :class:`ros_bt_py.ros_helpers.AsyncServiceProxy`
    behind the scenes.

    Due to the way that class works (it uses the `multiprocessing`
    module), the first tick of this node will almost certainly leave it
    in the RUNNING state, even if the service responds very quicly.

    If this node is ticked again after having returned SUCCEEDED or
    FAILED, it will call the service again with the now current request
    data.
    """

    _request_type: type
    _response_type: type
    _service_client: Optional[Client] = None

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

        self._service_type = self.options["service_type"].get_type_obj()

        node_inputs = {}
        node_outputs = {}
        try:
            self._request_type = getattr(self._service_type, "Request")
            if inspect.isclass(self._request_type):
                msg = self._request_type()
                for field in msg._fields_and_field_types:
                    node_inputs[field] = type(getattr(msg, field))
                self.passthrough = False
            else:
                node_inputs["in"] = self._service_type
        except AttributeError:
            node_inputs["in"] = self._service_type
            self.logwarn(f"Non message type passed to: {self.name}")

        try:
            self._response_type = getattr(self._service_type, "Response")
            if inspect.isclass(self._response_type):
                msg = self._response_type()
                for field in msg._fields_and_field_types:
                    node_outputs[field] = type(getattr(msg, field))
                self.passthrough = False
            else:
                node_outputs["out"] = self._service_type
        except AttributeError:
            node_outputs["out"] = self._service_type
            self.logwarn(f"Non message type passed to: {self.name}")

        self.node_config.extend(
            NodeConfig(
                options={}, inputs=node_inputs, outputs=node_outputs, max_children=0
            )
        )

        self._register_node_data(source_map=node_inputs, target_map=self.inputs)
        self._register_node_data(source_map=node_outputs, target_map=self.outputs)

    def _do_setup(self):
        self._service_client: Optional[Client] = None
        self._service_request_future: Optional[Future] = None
        self._reported_result: bool = False
        for k, v in self._response_type.get_fields_and_field_types().items():
            self.outputs[k] = None

        if not self.has_ros_node:
            raise BehaviorTreeException(
                "ROS service node does not have ROS node reference!"
            )

        return NodeMsg.IDLE

    def _do_reset(self):
        if self._service_client is not None:
            self.ros_node.destroy_client(self._service_client)
            self._service_client = None

        self._last_service_call_time: Optional[Time] = None
        self._last_request = None
        for k, v in self._response_type.get_fields_and_field_types().items():
            self.outputs[k] = None

        return NodeMsg.IDLE

    def _do_tick(self):
        if self.simulate_tick:
            self.logdebug(f"Simulating tick. {self.name} is not executing!")
            if self.succeed_always:
                return NodeMsg.SUCCEEDED

            return NodeMsg.RUNNING

        # If the service name changed
        if self.inputs.is_updated("service_name"):
            if self._service_client is not None:
                self._do_reset()
        if self._service_client is None:
            if self.has_ros_node:
                self._service_client = self.ros_node.create_client(
                    self._service_type,
                    self.inputs["service_name"],
                    callback_group=ReentrantCallbackGroup(),
                )
            else:
                self.logerr(f"No ROS node available for node: {self.name}!")
                return NodeMsg.FAILED
        # If theres' no service call in-flight, and we have already reported
        # the result (see below), start a new call and save the request
        if self._service_request_future is None:
            self._last_request = self._request_type()
            fields: dict[
                str, type
            ] = self._last_request.get_fields_and_field_types().items()
            for k, v in fields:
                setattr(self._last_request, k, self.inputs[k])

            self._reported_result = False
            self._last_service_call_time = self.ros_node.get_clock().now()
            self._service_request_future = self._service_client.call_async(
                self._last_request
            )

        if self._service_request_future is not None and not (
            self._service_request_future.done()
            or self._service_request_future.cancelled()
        ):
            # If the call takes longer than the specified timeout, abort the
            # call and return FAILED
            if self._last_service_call_time is None:
                self.logdebug(
                    "No previous timeout start timestamp set! Timeout starts now"
                )
                self._last_service_call_time = self.ros_node.get_clock().now()

            seconds_since_call: Duration = (
                self.ros_node.get_clock().now() - self._last_service_call_time
            ).nanoseconds / 1e9

            if seconds_since_call > self.options["wait_for_response_seconds"]:
                self.logwarn(
                    f"Service call to {self.inputs['service_name']} with request "
                    f"{self._last_request} timed out after {seconds_since_call} seconds"
                )
                self._service_request_future.cancel()
                return NodeMsg.FAILED

            return NodeMsg.RUNNING
        else:
            new_state = NodeMsg.SUCCEEDED
            if self._service_request_future.done():
                res = self._service_request_future.result()
                fields = res.get_fields_and_field_types().items()
                for k, v in fields:
                    self.outputs[k] = getattr(res, k)
            if self._service_request_future.cancelled():
                new_state = NodeMsg.FAILED
            self._service_request_future = None
            self._reported_result = True
            return new_state

    def _do_untick(self):
        if (
            self._service_request_future is not None
            and not self._service_request_future.done()
        ):
            self._service_request_future.cancel()
        self._service_request_future = None
        return NodeMsg.IDLE

    def _do_shutdown(self):
        self._service_request_future = None
        if self._service_client is not None:
            self.ros_node.destroy_client(self._service_client)

    def _do_calculate_utility(self):
        if not self.has_ros_node or self._service_client is None:
            self.logdebug(
                f"Unable to check for service {self.inputs['service_name']}, "
                "ros node available!"
            )
            return UtilityBounds()

        if self._service_client.service_is_ready():
            self.logdebug(
                f"Found service {self.inputs['service_name']} with correct type, returning "
                "filled out UtilityBounds"
            )
            return UtilityBounds(
                can_execute=True,
                has_lower_bound_success=True,
                has_upper_bound_success=True,
                has_lower_bound_failure=True,
                has_upper_bound_failure=True,
            )

        self.logdebug(f"Service {self.inputs['service_name']} is unavailable")
        return UtilityBounds(can_execute=False)


@define_bt_node(
    NodeConfig(
        version="0.1.0",
        options={
            "service_type": RosServiceType,
            "service_name": RosServiceName,
            "wait_for_service_seconds": float,
        },
        inputs={},
        outputs={},
        max_children=0,
        optional_options=[],
    )
)
class WaitForService(Leaf):
    """Wait for a service to be available, fails if this wait times out."""

    def __init__(
        self, 
        options = None, 
        debug_manager = None, 
        subtree_manager = None, 
        name = None, 
        ros_node = None, 
        succeed_always = False, 
        simulate_tick = False
    ):
        super().__init__(
            options, 
            debug_manager, 
            subtree_manager, 
            name, 
            ros_node, 
            succeed_always, 
            simulate_tick
        )

        self._service_type = self.options["service_type"].get_type_obj()
        self._service_name = self.options["service_name"].name

    def _do_setup(self):
        if not self.has_ros_node:
            self.logerr("Not ROS node reference available!")
            raise BehaviorTreeException("No ROS node reference available!")

        self._service_client = self.ros_node.create_client(
            self._service_name, self._service_type
        )
        self._last_service_call_time: Optional[Time] = None

    def _do_tick(self):
        if self.simulate_tick:
            self.logdebug(f"Simulating tick. {self.name} is not executing!")
            if self.succeed_always:
                return NodeMsg.SUCCEEDED

            return NodeMsg.RUNNING

        if self._service_client.service_is_ready():
            return NodeMsg.SUCCEEDED
        else:
            if self._last_service_call_time is None:
                self._last_service_call_time = self.ros_node.get_clock().now()
            elapsed_time: Duration = (
                self.ros_node.get_clock().now() - self._last_service_call_time
            )

            if (elapsed_time.nanoseconds / 1e9) > self.options[
                "wait_for_service_seconds"
            ]:
                return NodeMsg.FAILED
            else:
                return NodeMsg.RUNNING

    def _do_untick(self):
        self._last_service_call_time = None
        return NodeMsg.IDLE

    def _do_reset(self):
        self._last_service_call_time = None
        return NodeMsg.IDLE

    def _do_shutdown(self):
        self._ros_node.destroy_client(self._service_client)


@define_bt_node(
    NodeConfig(
        version="0.1.0",
        options={"service_type": RosServiceType, "wait_for_service_seconds": float},
        inputs={"service_name": str},
        outputs={},
        max_children=0,
        optional_options=[],
    )
)
class WaitForServiceInput(Leaf):
    """Wait for a service to be available, fails if this wait times out."""

    def __init__(
        self, 
        options = None, 
        debug_manager = None, 
        subtree_manager = None, 
        name = None, 
        ros_node = None, 
        succeed_always = False, 
        simulate_tick = False
    ):
        super().__init__(
            options, 
            debug_manager, 
            subtree_manager, 
            name, 
            ros_node, 
            succeed_always, 
            simulate_tick
        )

        self._service_type = self.options["service_type"].get_type_obj()

    def _do_setup(self):
        if not self.has_ros_node:
            self.logerr("Not ROS node reference available!")
            raise BehaviorTreeException("No ROS node reference available!")

        self._service_client: Optional[Client] = None
        self._last_service_call_time: Optional[Time] = None

    def _do_tick(self):
        if self.simulate_tick:
            self.logdebug(f"Simulating tick. {self.name} is not executing!")
            if self.succeed_always:
                return NodeMsg.SUCCEEDED

            return NodeMsg.RUNNING

        if self._service_client is None:
            self._service_client = self.ros_node.create_client(
                self.inputs["service_name"], self._service_type
            )

        if self._service_client.service_is_ready():
            return NodeMsg.SUCCEEDED
        else:
            if self._last_service_call_time is None:
                self._last_service_call_time = self.ros_node.get_clock().now()
            elapsed_time: Duration = (
                self.ros_node.get_clock().now() - self._last_service_call_time
            )

            if (elapsed_time.nanoseconds / 1e9) > self.options[
                "wait_for_service_seconds"
            ]:
                return NodeMsg.FAILED
            else:
                return NodeMsg.RUNNING

    def _do_untick(self):
        self._last_service_call_time = None
        return NodeMsg.IDLE

    def _do_reset(self):
        if self._service_client is not None:
            self.ros_node.destroy_service(self._service_client)
            self._service_client = None
        self._last_service_call_time = None
        return NodeMsg.IDLE

    def _do_shutdown(self):
        self._last_service_call_time = None
        if self._service_client is not None:
            self.ros_node.destroy_client(self._service_client)
            self._service_client = None


@define_bt_node(
    NodeConfig(
        options={
            "service_name": RosServiceName,
            "wait_for_service_seconds": float,
            "wait_for_response_seconds": float,
            "fail_if_not_available": bool,
        },
        inputs={},
        outputs={},
        max_children=0,
        optional_options=["fail_if_not_available"],
    )
)
class ServiceForSetType(ABC, Leaf):
    """
    Abstract ROS service class.

    Inherit form this class to create a ROS service node with
    a defined service type and build a cleaner Behavior Tree.

    Due to the way that class works (it uses the `multiprocessing`
    module), the first tick of this node will almost certainly leave it
    in the RUNNING state, even if the service responds very quickly.

    If this node is ticked again after having returned SUCCEEDED or
    FAILED, it will call the service again with the now current request
    data.

    Example:
    -------
        >>> @define_bt_node(NodeConfig(
                options={'MyOption' : MyOptionType},
                inputs ={'MyInput' : MyInputType},
                outputs={'MyOutput' : MyOutputType},
                max_children=0,
                optional_options=['fail_if_not_available']))
        >>> class CallMyService(ServiceForSetType):

                # Set the service type
                def set_service_type(self):
                    self._service_type = MyServiceType

                # Set all outputs to none (define output key while overwriting)
                def set_output_none(self):
                    self.outputs['MyServiceOutput'] = None

                # Sets the service request message, sent to the service.
                def set_request(self):
                    self._last_request = MyServiceRequest()

                # Sets the output (in relation to the response)
                # (define output key while overwriting)
                # it should return True, if the node state should be SUCCEEDED after receiving
                #the message and False. if it should be in the FAILED state
                def set_outputs(self):
                    self.outputs['MyServiceOutput'] =\
                    self._service_proxy.get_response().MyServiceOutput
                    return self.outputs[’MyServiceOutput']

    """

    _service_request_future: Optional[Future] = None
    _last_service_call_time: Optional[Time] = None
    _last_request: Optional[Any] = None
    _reported_result: bool = False
    _service_name: str

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
        self._service_client: Optional[Client] = None
        self._service_name = self.options["service_name"].name
        self.set_service_type()

    # Sets all outputs none (define output key while overwriting)
    @abstractmethod
    def set_output_none(self):
        self.outputs["OUTPUT_KEY"] = None

    # Returns the service request message that should be send to the service.
    @abstractmethod
    def set_request(self):
        pass

    # Sets the output (in relation to the response) (define output key while overwriting)
    # Should return True, if the node state should be SUCCEEDED after receiving the message
    # and False, if it's in the FAILED state
    @abstractmethod
    def set_outputs(self) -> bool:
        if self._service_request_future is not None:
            self.outputs["OUTPUT_KEY"] = self._service_request_future.result()
            return True
        else:
            return False

    # Sets the service type
    @abstractmethod
    def set_service_type(self):
        self._service_type = "SERVICE_TYPE"

    def _do_setup(self):
        self._service_available = True

        if not self.has_ros_node:
            raise BehaviorTreeException(
                f"ROS node reference not available for {self.name}!"
            )
        self._service_client = self.ros_node.create_client(
            self._service_type,
            self._service_name,
            callback_group=ReentrantCallbackGroup(),
        )
        # Exception if service is not available
        available = self._service_client.wait_for_service(
            timeout_sec=self.options["wait_for_service_seconds"]
        )
        if (
            not available
            and "fail_if_not_available" in self.options
            and self.options["fail_if_not_available"]
        ):
            self._service_available = False
            return NodeMsg.BROKEN

        self._last_service_call_time: Optional[Time] = None
        self._service_request_future: Optional[Future] = None
        self._last_request = None
        self._reported_result: bool = False
        self.set_output_none()
        return NodeMsg.IDLE

    def _do_reset(self):
        if (
            self._service_request_future is not None
            and not self._service_request_future.done()
        ):
            self._service_request_future.cancel()
        self._service_request_future = None
        self._last_service_call_time = None
        self._last_request = None
        self._reported_result: bool = False
        self.set_output_none()
        return NodeMsg.IDLE

    def _do_tick(self):
        if self.simulate_tick:
            self.logdebug(f"Simulating tick. {self.name} is not executing!")
            if self.succeed_always:
                return NodeMsg.SUCCEEDED

            return NodeMsg.RUNNING

        if not self._service_available or self._service_client is None:
            return NodeMsg.FAILED
        # If theres' no service call in-flight, and we have already reported
        # the result (see below), start a new call and save the request
        if self._service_request_future is None:
            self.logdebug("Future is None, starting new request!")
            self._reported_result = False
            self.set_request()
            self._last_service_call_time = self.ros_node.get_clock().now()
            self._service_request_future = self._service_client.call_async(
                self._last_request
            )
            self.logdebug(f"Request future: {self._service_request_future}")

        if self._service_request_future is None:
            self.logerr("Service request future is unexpecedly none!")
            return NodeMsg.FAILED

        if self._service_request_future.cancelled():
            self.logdebug("Service request was cancelled!")
            self._service_request_future = None
            return NodeMsg.FAILURE

        if not self._service_request_future.done():
            # If the call takes longer than the specified timeout, abort the
            # call and return FAILED
            if self._last_service_call_time is None:
                self.logwarn(
                    "No previous timeout start timestamp set! Timeout starts now"
                )
                self._last_service_call_time = self.ros_node.get_clock().now()

            seconds_since_call: float = (
                self.ros_node.get_clock().now() - self._last_service_call_time
            ).nanoseconds / 1e9
            if seconds_since_call > self.options["wait_for_response_seconds"]:
                self.loginfo(
                    f"Service call to {self._service_name} with request "
                    f"{self._last_request} timed out"
                )
                self._service_request_future.cancel()
                self._service_request_future = None
                return NodeMsg.FAILED

            return NodeMsg.RUNNING
        else:
            if self.set_outputs():
                new_state = NodeMsg.SUCCEEDED
            else:
                new_state = NodeMsg.FAILED
            self._reported_result = True
            self._service_request_future = None
            return new_state

    def _do_untick(self):
        if (
            self._service_request_future is not None
            and not self._service_request_future.done()
        ):
            self._service_request_future.cancel()
        return NodeMsg.IDLE

    def _do_shutdown(self):
        self._do_reset()
        if self._service_client is not None:
            self.ros_node.destroy_client(self._service_client)
        self._service_client = None

    def _do_calculate_utility(self):
        if not self.has_ros_node or self._service_client is None:
            self.logdebug(
                f"Unable to check for service {self._service_name}: "
                "No ros node available!"
            )
            return UtilityBounds(can_execute=False)

        if self._service_client.service_is_ready():
            self.logdebug(
                f"Found service {self._service_name} with correct type, returning "
                "filled out UtilityBounds"
            )
            return UtilityBounds(
                can_execute=True,
                has_lower_bound_success=True,
                has_upper_bound_success=True,
                has_lower_bound_failure=True,
                has_upper_bound_failure=True,
            )

        self.logdebug(f"Service {self._service_name} is unavailable")
        return UtilityBounds(can_execute=False)


@define_bt_node(
    NodeConfig(
        options={
            "service_name": RosServiceName,
            "service_type": RosServiceType,
            "wait_for_service_seconds": float,
            "wait_for_response_seconds": float,
            "fail_if_not_available": bool,
        },
        inputs={},
        outputs={},
        max_children=0,
        optional_options=["fail_if_not_available"],
    )
)
class Service(Leaf):
    """
    Call a ROS service with the provided Request data.

    To make sure the service call cannot block the :meth:`tick()`
    method, this uses a :class:`ros_bt_py.ros_helpers.AsyncServiceProxy`
    behind the scenes.
    Due to the way that class works (it uses the `multiprocessing`
    module), the first tick of this node will almost certainly leave it
    in the RUNNING state, even if the service responds very quicly.

    If this node is ticked again after having returned SUCCEEDED or
    FAILED, it will call the service again with the now current request
    data.
    """

    _service_client: Optional[Client] = None
    _service_request_future: Optional[Future] = None
    _last_service_call_time: Optional[Time] = None
    _last_request: Optional[Any] = None
    _reported_result: bool = False
    _service_name: str
    _request_type: type
    _response_type: type

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

        self._service_type = self.options["service_type"].get_type_obj()
        self._service_name = self.options["service_name"].name

        node_inputs = {}
        node_outputs = {}
        try:
            self._request_type = getattr(self._service_type, "Request")
            if inspect.isclass(self._request_type):
                msg = self._request_type()
                for field in msg._fields_and_field_types:
                    node_inputs[field] = type(getattr(msg, field))
                self.passthrough = False
            else:
                node_inputs["in"] = self._service_type
        except AttributeError:
            node_inputs["in"] = self._service_type
            self.logwarn(f"Non message type passed to: {self.name}")

        try:
            self._response_type = getattr(self._service_type, "Response")
            if inspect.isclass(self._response_type):
                msg = self._response_type()
                for field in msg._fields_and_field_types:
                    node_outputs[field] = type(getattr(msg, field))
                self.passthrough = False
            else:
                node_outputs["out"] = self._service_type
        except AttributeError:
            node_outputs["out"] = self._service_type
            self.logwarn(f"Non message type passed to: {self.name}")

        self.node_config.extend(
            NodeConfig(
                options={}, inputs=node_inputs, outputs=node_outputs, max_children=0
            )
        )

        self._register_node_data(source_map=node_inputs, target_map=self.inputs)
        self._register_node_data(source_map=node_outputs, target_map=self.outputs)

    def _do_setup(self):
        self._service_client: Optional[Client] = None
        self._service_request_future: Optional[Future] = None
        self._reported_result: bool = False
        for k, v in self._response_type.get_fields_and_field_types().items():
            self.outputs[k] = None

        if not self.has_ros_node:
            raise BehaviorTreeException(
                "ROS service node does not have ROS node reference!"
            )

        return NodeMsg.IDLE

    def _do_reset(self):
        if self._service_client is not None:
            self.ros_node.destroy_client(self._service_client)
            self._service_client = None

        self._last_service_call_time: Optional[Time] = None
        self._last_request = None
        self._reported_result = False
        for k, v in self._response_type.get_fields_and_field_types().items():
            self.outputs[k] = None

        return NodeMsg.IDLE

    def _do_tick(self):
        if self.simulate_tick:
            self.logdebug(f"Simulating tick. {self.name} is not executing!")
            if self.succeed_always:
                return NodeMsg.SUCCEEDED

            return NodeMsg.RUNNING

        #TODO Can't this be in `_do_setup`?
        if self._service_client is None:
            if self.has_ros_node:
                self._service_client = self.ros_node.create_client(
                    self._service_type,
                    self._service_name,
                    callback_group=ReentrantCallbackGroup(),
                )
            else:
                self.logerr(f"No ROS node available for node: {self.name}!")
                return NodeMsg.FAILED
        # If theres' no service call in-flight, and we have already reported
        # the result (see below), start a new call and save the request
        if self._service_request_future is None:
            self._last_request = self._request_type()
            fields: dict[
                str, type
            ] = self._last_request.get_fields_and_field_types().items()
            for k, v in fields:
                setattr(self._last_request, k, self.inputs[k])

            self._last_service_call_time = self.ros_node.get_clock().now()
            self._service_request_future = self._service_client.call_async(
                self._last_request
            )

        if self._service_request_future is not None and not (
            self._service_request_future.done()
            or self._service_request_future.cancelled()
        ):
            # If the call takes longer than the specified timeout, abort the
            # call and return FAILED
            if self._last_service_call_time is None:
                self.logwarn(
                    "No previous timeout start timestamp set! Timeout starts now"
                )
                self._last_service_call_time = self.ros_node.get_clock().now()

            seconds_since_call: Duration = (
                self.ros_node.get_clock().now() - self._last_service_call_time
            ).nanoseconds / 1e9

            if seconds_since_call > self.options["wait_for_response_seconds"]:
                self.logerr(
                    f"Service call to {self._service_name} with request "
                    f"{self._last_request} timed out after {seconds_since_call} seconds"
                )
                self._service_request_future.cancel()
                return NodeMsg.FAILED

            return NodeMsg.RUNNING
        else:
            new_state = NodeMsg.SUCCEEDED
            if self._service_request_future.done():
                res = self._service_request_future.result()
                fields = res.get_fields_and_field_types().items()
                for k, v in fields:
                    self.outputs[k] = getattr(res, k)

            if self._service_request_future.cancelled():
                new_state = NodeMsg.FAILED

            self._service_request_future = None
            self._reported_result = True
            return new_state

    def _do_untick(self):
        if (
            self._service_request_future is not None
            and not self._service_request_future.done()
        ):
            self._service_request_future.cancel()
        return NodeMsg.IDLE

    def _do_shutdown(self):
        if self._service_client is not None:
            self.ros_node.destroy_client(self._service_client)

    def _do_calculate_utility(self):
        if not self.has_ros_node or self._service_client is None:
            self.loginfo(
                f"Unable to check for service {self._service_name}, "
                "ros node available!"
            )
            return UtilityBounds()

        if self._service_client.service_is_ready():
            self.loginfo(
                f"Found service {self._service_name} with correct type, returning "
                "filled out UtilityBounds"
            )
            return UtilityBounds(
                can_execute=True,
                has_lower_bound_success=True,
                has_upper_bound_success=True,
                has_lower_bound_failure=True,
                has_upper_bound_failure=True,
            )

        self.loginfo(f"Service {self._service_name} is unavailable")
        return UtilityBounds(can_execute=False)
