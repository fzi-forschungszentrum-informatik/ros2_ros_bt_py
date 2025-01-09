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
from typeguard import typechecked
from rclpy.task import Future
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.client import Client
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time

from ros_bt_py.helpers import BTNodeState
from ros_bt_py_interfaces.msg import UtilityBounds

from ros_bt_py.debug_manager import DebugManager
from ros_bt_py.subtree_manager import SubtreeManager
from ros_bt_py.node import Leaf, define_bt_node
from ros_bt_py.node_config import NodeConfig
from ros_bt_py.exceptions import BehaviorTreeException

from abc import ABC, abstractmethod
from typing import Any, Optional, Dict

import inspect

from result import Result, Ok, Err, is_err


@define_bt_node(
    NodeConfig(
        version="0.1.0",
        options={
            "service_type": type,
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
    ) -> None:
        super().__init__(
            options=options,
            debug_manager=debug_manager,
            subtree_manager=subtree_manager,
            name=name,
            ros_node=ros_node,
        )

        node_inputs: Dict[str, Any] = {}
        node_outputs: Dict[str, Any] = {}
        try:
            self._request_type = getattr(self.options["service_type"], "Request")
            if inspect.isclass(self._request_type):
                msg = self._request_type()
                for field in msg._fields_and_field_types:
                    node_inputs[field] = type(getattr(msg, field))
                self.passthrough = False
            else:
                node_inputs["in"] = self.options["service_type"]
        except AttributeError:
            node_inputs["in"] = self.options["service_type"]
            self.logwarn(f"Non message type passed to: {self.name}")

        try:
            self._response_type = getattr(self.options["service_type"], "Response")
            if inspect.isclass(self._response_type):
                msg = self._response_type()
                for field in msg._fields_and_field_types:
                    node_outputs[field] = type(getattr(msg, field))
                self.passthrough = False
            else:
                node_outputs["out"] = self.options["service_type"]
        except AttributeError:
            node_outputs["out"] = self.options["service_type"]
            self.logwarn(f"Non message type passed to: {self.name}")

        node_config_extend_result = self.node_config.extend(
            NodeConfig(
                options={}, inputs=node_inputs, outputs=node_outputs, max_children=0
            )
        )
        if node_config_extend_result.is_err():
            self.state = BTNodeState.BROKEN
            return

        input_result = self._register_node_data(
            source_map=node_inputs, target_map=self.inputs
        )
        output_result = self._register_node_data(
            source_map=node_outputs, target_map=self.outputs
        )
        if input_result.is_err() or output_result.is_err():
            self.state = BTNodeState.BROKEN
            return

    def _do_setup(self) -> Result[BTNodeState, BehaviorTreeException]:
        self._service_client: Optional[Client] = None
        self._service_request_future: Optional[Future] = None
        self._reported_result: bool = False
        for k, v in self._response_type.get_fields_and_field_types().items():
            self.outputs[k] = None

        if not self.has_ros_node:
            return Err(
                BehaviorTreeException(
                    "ROS service node does not have ROS node reference!"
                )
            )

        return Ok(BTNodeState.IDLE)

    def _do_reset(self) -> Result[BTNodeState, BehaviorTreeException]:
        if self._service_client is not None:
            self.ros_node.destroy_client(self._service_client)
            self._service_client = None

        self._last_service_call_time: Optional[Time] = None
        self._last_request = None
        for k, v in self._response_type.get_fields_and_field_types().items():
            self.outputs[k] = None

        return Ok(BTNodeState.IDLE)

    def _do_tick(self) -> Result[BTNodeState, BehaviorTreeException]:
        # If the service name changed
        if self.inputs.is_updated("service_name"):
            if self._service_client is not None:
                self._do_reset()
        if self._service_client is None:
            if self.has_ros_node:
                self._service_client = self.ros_node.create_client(
                    self.options["service_type"],
                    self.inputs["service_name"],
                    callback_group=ReentrantCallbackGroup(),
                )
            else:
                self.logerr(f"No ROS node available for node: {self.name}!")
                return Ok(BTNodeState.FAILED)
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

        if (
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

            seconds_since_call: float = (
                self.ros_node.get_clock().now() - self._last_service_call_time
            ).nanoseconds / 1e9

            if seconds_since_call > self.options["wait_for_response_seconds"]:
                self.logwarn(
                    f"Service call to {self.inputs['service_name']} with request "
                    f"{self._last_request} timed out after {seconds_since_call} seconds"
                )
                self._service_request_future.cancel()
                return Ok(BTNodeState.FAILED)

            return Ok(BTNodeState.RUNNING)
        else:
            new_state = BTNodeState.SUCCEEDED
            if self._service_request_future.done():
                res = self._service_request_future.result()
                if res is None:
                    return Err(BehaviorTreeException("Service response is none!"))
                fields = res.get_fields_and_field_types().items()
                for k, v in fields:
                    self.outputs[k] = getattr(res, k)
            if self._service_request_future.cancelled():
                new_state = BTNodeState.FAILED
            self._service_request_future = None
            self._reported_result = True
            return Ok(new_state)

    def _do_untick(self) -> Result[BTNodeState, BehaviorTreeException]:
        if (
            self._service_request_future is not None
            and not self._service_request_future.done()
        ):
            self._service_request_future.cancel()
        self._service_request_future = None
        return Ok(BTNodeState.IDLE)

    def _do_shutdown(self) -> Result[BTNodeState, BehaviorTreeException]:
        self._service_request_future = None
        if self._service_client is not None:
            self.ros_node.destroy_client(self._service_client)
        return Ok(BTNodeState.SHUTDOWN)

    def _do_calculate_utility(self) -> Result[UtilityBounds, BehaviorTreeException]:
        if not self.has_ros_node or self._service_client is None:
            self.logdebug(
                f"Unable to check for service {self.inputs['service_name']}, "
                "ros node available!"
            )
            return Ok(UtilityBounds())

        if self._service_client.service_is_ready():
            self.logdebug(
                f"Found service {self.inputs['service_name']} with correct type, returning "
                "filled out UtilityBounds"
            )
            return Ok(
                UtilityBounds(
                    can_execute=True,
                    has_lower_bound_success=True,
                    has_upper_bound_success=True,
                    has_lower_bound_failure=True,
                    has_upper_bound_failure=True,
                )
            )

        self.logdebug(f"Service {self.inputs['service_name']} is unavailable")
        return Ok(UtilityBounds(can_execute=False))


@define_bt_node(
    NodeConfig(
        version="0.1.0",
        options={
            "service_type": type,
            "service_name": str,
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

    def _do_setup(self) -> Result[BTNodeState, BehaviorTreeException]:
        if not self.has_ros_node:
            self.logerr("Not ROS node reference available!")
            return Err(BehaviorTreeException("No ROS node reference available!"))

        self._service_client = self.ros_node.create_client(
            self.options["service_name"], self.options["service_type"]
        )
        self._last_service_call_time: Optional[Time] = None
        return Ok(BTNodeState.IDLE)

    def _do_tick(self) -> Result[BTNodeState, BehaviorTreeException]:

        if self._service_client.service_is_ready():
            return Ok(BTNodeState.SUCCEEDED)
        else:
            if self._last_service_call_time is None:
                self._last_service_call_time = self.ros_node.get_clock().now()
            elapsed_time: Duration = (
                self.ros_node.get_clock().now() - self._last_service_call_time
            )

            if (elapsed_time.nanoseconds / 1e9) > self.options[
                "wait_for_service_seconds"
            ]:
                return Ok(BTNodeState.FAILED)
            else:
                return Ok(BTNodeState.RUNNING)

    def _do_untick(self) -> Result[BTNodeState, BehaviorTreeException]:
        self._last_service_call_time = None
        return Ok(BTNodeState.IDLE)

    def _do_reset(self) -> Result[BTNodeState, BehaviorTreeException]:
        self._last_service_call_time = None
        return Ok(BTNodeState.IDLE)

    def _do_shutdown(self) -> Result[BTNodeState, BehaviorTreeException]:
        if self.has_ros_node and self._ros_node.destroy_client(self._service_client):
            return Ok(BTNodeState.SHUTDOWN)
        else:
            return Err(
                BehaviorTreeException(
                    f"Failed to destory service handle in {self.name}"
                )
            )


@define_bt_node(
    NodeConfig(
        version="0.1.0",
        options={"service_type": type, "wait_for_service_seconds": float},
        inputs={"service_name": str},
        outputs={},
        max_children=0,
        optional_options=[],
    )
)
class WaitForServiceInput(Leaf):
    """Wait for a service to be available, fails if this wait times out."""

    def _do_setup(self) -> Result[BTNodeState, BehaviorTreeException]:
        if not self.has_ros_node:
            self.logerr("Not ROS node reference available!")
            return Err(BehaviorTreeException("No ROS node reference available!"))

        self._service_client: Optional[Client] = None
        self._last_service_call_time: Optional[Time] = None
        return Ok(BTNodeState.IDLE)

    def _do_tick(self) -> Result[BTNodeState, BehaviorTreeException]:
        if self._service_client is None:
            self._service_client = self.ros_node.create_client(
                self.inputs["service_name"], self.options["service_type"]
            )

        if self._service_client.service_is_ready():
            return Ok(BTNodeState.SUCCEEDED)
        else:
            if self._last_service_call_time is None:
                self._last_service_call_time = self.ros_node.get_clock().now()
            elapsed_time: Duration = (
                self.ros_node.get_clock().now() - self._last_service_call_time
            )

            if (elapsed_time.nanoseconds / 1e9) > self.options[
                "wait_for_service_seconds"
            ]:
                return Ok(BTNodeState.FAILED)
            else:
                return Ok(BTNodeState.RUNNING)

    def _do_untick(self) -> Result[BTNodeState, BehaviorTreeException]:
        self._last_service_call_time = None
        return Ok(BTNodeState.IDLE)

    def _do_reset(self) -> Result[BTNodeState, BehaviorTreeException]:
        if self._service_client is not None:
            if not self.ros_node.destroy_client(self._service_client):
                return Err(BehaviorTreeException("Could not destroy service client!"))
            self._service_client = None
        self._last_service_call_time = None
        return Ok(BTNodeState.IDLE)

    def _do_shutdown(self) -> Result[BTNodeState, BehaviorTreeException]:
        self._last_service_call_time = None
        if self._service_client is not None:
            if not self.ros_node.destroy_client(self._service_client):
                return Err(BehaviorTreeException("Could not destroy service client!"))
            self._service_client = None
        return Ok(BTNodeState.SHUTDOWN)


@define_bt_node(
    NodeConfig(
        options={
            "service_name": str,
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
    ) -> None:
        super().__init__(
            options=options,
            debug_manager=debug_manager,
            subtree_manager=subtree_manager,
            name=name,
            ros_node=ros_node,
        )
        self._service_client: Optional[Client] = None
        self._service_name = self.options["service_name"]
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

    def _do_setup(self) -> Result[BTNodeState, BehaviorTreeException]:
        self._service_available = True

        if not self.has_ros_node:
            return Err(
                BehaviorTreeException(
                    f"ROS node reference not available for {self.name}!"
                )
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
            return Ok(BTNodeState.BROKEN)

        self._last_service_call_time: Optional[Time] = None
        self._service_request_future: Optional[Future] = None
        self._last_request = None
        self._reported_result: bool = False
        self.set_output_none()
        return Ok(BTNodeState.IDLE)

    def _do_reset(self) -> Result[BTNodeState, BehaviorTreeException]:
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
        return Ok(BTNodeState.IDLE)

    def _do_tick(self) -> Result[BTNodeState, BehaviorTreeException]:

        if not self._service_available or self._service_client is None:
            return Ok(BTNodeState.FAILED)
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
            return Ok(BTNodeState.FAILED)

        if self._service_request_future.cancelled():
            self.logdebug("Service request was cancelled!")
            self._service_request_future = None
            return Ok(BTNodeState.FAILED)

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
                    f"Service call to {self.options['service_name']} with request "
                    f"{self._last_request} timed out"
                )
                self._service_request_future.cancel()
                self._service_request_future = None
                return Ok(BTNodeState.FAILED)

            return Ok(BTNodeState.RUNNING)
        else:
            if self.set_outputs():
                new_state = BTNodeState.SUCCEEDED
            else:
                new_state = BTNodeState.FAILED
            self._reported_result = True
            self._service_request_future = None
            return Ok(new_state)

    def _do_untick(self) -> Result[BTNodeState, BehaviorTreeException]:
        if (
            self._service_request_future is not None
            and not self._service_request_future.done()
        ):
            self._service_request_future.cancel()
        return Ok(BTNodeState.IDLE)

    def _do_shutdown(self) -> Result[BTNodeState, BehaviorTreeException]:
        reset_result = self._do_reset()
        if reset_result.is_err():
            return reset_result

        if self._service_client is not None:
            self.ros_node.destroy_client(self._service_client)
        self._service_client = None
        return Ok(BTNodeState.SHUTDOWN)

    def _do_calculate_utility(self) -> Result[UtilityBounds, BehaviorTreeException]:
        if not self.has_ros_node or self._service_client is None:
            self.logdebug(
                f"Unable to check for service {self.options['service_name']}: "
                "No ros node available!"
            )
            return Ok(UtilityBounds(can_execute=False))

        if self._service_client.service_is_ready():
            self.logdebug(
                f"Found service {self.options['service_name']} with correct type, returning "
                "filled out UtilityBounds"
            )
            return Ok(
                UtilityBounds(
                    can_execute=True,
                    has_lower_bound_success=True,
                    has_upper_bound_success=True,
                    has_lower_bound_failure=True,
                    has_upper_bound_failure=True,
                )
            )

        self.logdebug(f"Service {self.options['service_name']} is unavailable")
        return Ok(UtilityBounds(can_execute=False))


@define_bt_node(
    NodeConfig(
        options={
            "service_name": str,
            "service_type": type,
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
    ) -> None:
        super().__init__(
            options=options,
            debug_manager=debug_manager,
            subtree_manager=subtree_manager,
            name=name,
            ros_node=ros_node,
        )

        node_inputs = {}
        node_outputs = {}
        try:
            self._request_type = getattr(self.options["service_type"], "Request")
            if inspect.isclass(self._request_type):
                msg = self._request_type()
                for field in msg._fields_and_field_types:
                    node_inputs[field] = type(getattr(msg, field))
                self.passthrough = False
            else:
                node_inputs["in"] = self.options["service_type"]
        except AttributeError:
            node_inputs["in"] = self.options["service_type"]
            self.logwarn(f"Non message type passed to: {self.name}")

        try:
            self._response_type = getattr(self.options["service_type"], "Response")
            if inspect.isclass(self._response_type):
                msg = self._response_type()
                for field in msg._fields_and_field_types:
                    node_outputs[field] = type(getattr(msg, field))
                self.passthrough = False
            else:
                node_outputs["out"] = self.options["service_type"]
        except AttributeError:
            node_outputs["out"] = self.options["service_type"]
            self.logwarn(f"Non message type passed to: {self.name}")

        # TODO: Use result type.
        extend_node_config_result = self.node_config.extend(
            NodeConfig(
                options={}, inputs=node_inputs, outputs=node_outputs, max_children=0
            )
        )
        if extend_node_config_result.is_err():
            self.state = BTNodeState.BROKEN
            self.logfatal(
                f"Failed to extend node config for {self.name}: "
                f"{extend_node_config_result.unwrap_err()}"
            )

        register_input_result = self._register_node_data(
            source_map=node_inputs, target_map=self.inputs
        )
        register_output_result = self._register_node_data(
            source_map=node_outputs, target_map=self.outputs
        )
        if register_input_result.is_err() or register_output_result.is_err():
            self.state = BTNodeState.BROKEN
            self.logfatal(f"Could not register input or output data for {self.name}")

    def _do_setup(self) -> Result[BTNodeState, BehaviorTreeException]:
        self._service_client: Optional[Client] = None
        self._service_request_future: Optional[Future] = None
        self._reported_result: bool = False
        for k, v in self._response_type.get_fields_and_field_types().items():
            self.outputs[k] = None

        if not self.has_ros_node:
            return Err(
                BehaviorTreeException(
                    "ROS service node does not have ROS node reference!"
                )
            )

        return Ok(BTNodeState.IDLE)

    def _do_reset(self) -> Result[BTNodeState, BehaviorTreeException]:
        if self._service_client is not None:
            if not self.ros_node.destroy_client(self._service_client):
                return Err(BehaviorTreeException("Could not destroy client!"))
            self._service_client = None

        self._last_service_call_time: Optional[Time] = None
        self._last_request = None
        self._reported_result = False
        for k, v in self._response_type.get_fields_and_field_types().items():
            self.outputs[k] = None

        return Ok(BTNodeState.IDLE)

    def _do_tick(self) -> Result[BTNodeState, BehaviorTreeException]:

        if self._service_client is None:
            if self.has_ros_node:
                self._service_client = self.ros_node.create_client(
                    self.options["service_type"],
                    self.options["service_name"],
                    callback_group=ReentrantCallbackGroup(),
                )
            else:
                msg = f"No ROS node available for node: {self.name}!"
                self.logerr(msg)
                return Err(BehaviorTreeException(msg))
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

            seconds_since_call: float = (
                self.ros_node.get_clock().now() - self._last_service_call_time
            ).nanoseconds / 1e9

            if seconds_since_call > self.options["wait_for_response_seconds"]:
                self.logwarn(
                    f"Service call to {self.options['service_name']} with request "
                    f"{self._last_request} timed out after {seconds_since_call} seconds"
                )
                self._service_request_future.cancel()
                return Ok(BTNodeState.FAILED)

            return Ok(BTNodeState.RUNNING)
        else:
            new_state = BTNodeState.SUCCEEDED
            if self._service_request_future.done():
                res = self._service_request_future.result()
                fields = res.get_fields_and_field_types().items()
                for k, v in fields:
                    self.outputs[k] = getattr(res, k)

            if self._service_request_future.cancelled():
                new_state = BTNodeState.FAILED

            self._service_request_future = None
            self._reported_result = True
            return Ok(new_state)

    def _do_untick(self) -> Result[BTNodeState, BehaviorTreeException]:
        if (
            self._service_request_future is not None
            and not self._service_request_future.done()
        ):
            self._service_request_future.cancel()
        return Ok(BTNodeState.IDLE)

    def _do_shutdown(self) -> Result[BTNodeState, BehaviorTreeException]:
        if self._service_client is not None:
            if not self.ros_node.destroy_client(self._service_client):
                return Err(BehaviorTreeException("Failed to destroy client!"))
        return Ok(BTNodeState.SHUTDOWN)

    def _do_calculate_utility(self) -> Result[UtilityBounds, BehaviorTreeException]:
        if not self.has_ros_node or self._service_client is None:
            msg = (
                f"Unable to check for service {self.options['service_name']}, "
                "ros node available!"
            )
            self.loginfo(msg)
            return Ok(UtilityBounds())

        if self._service_client.service_is_ready():
            self.loginfo(
                f"Found service {self.options['service_name']} with correct type, returning "
                "filled out UtilityBounds"
            )
            return Ok(
                UtilityBounds(
                    can_execute=True,
                    has_lower_bound_success=True,
                    has_upper_bound_success=True,
                    has_lower_bound_failure=True,
                    has_upper_bound_failure=True,
                )
            )

        self.loginfo(f"Service {self.options['service_name']} is unavailable")
        return Ok(UtilityBounds(can_execute=False))
