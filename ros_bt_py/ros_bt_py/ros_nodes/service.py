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
import abc
from typing import Any, Optional

from ros_bt_py.vendor.result import Result, Ok, Err, do

from rclpy.task import Future
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.client import Client
from rclpy.duration import Duration
from rclpy.time import Time

from ros_bt_py.data_types import (
    DataContainer,
    FloatType,
    RosServiceName,
    RosServiceType,
    get_message_field_io_type,
)
from ros_bt_py.exceptions import BehaviorTreeException, NodeConfigError
from ros_bt_py.helpers import BTNodeState
from ros_bt_py.node import Leaf, define_bt_node
from ros_bt_py.node_config import NodeConfig

from ros_bt_py_interfaces.msg import UtilityBounds


@define_bt_node(
    NodeConfig(
        inputs={
            "service_name": RosServiceName(interface_id=1),
            "wait_for_response_seconds": FloatType(allow_dynamic=False, value=10.2),
        },
        outputs={},
        max_children=0,
    )
)
class ServiceBase(Leaf):
    """
    Abstract ROS Service class.

    You can inherit from this class to define nodes with a specific service_type
    and custom inputs and outputs.

    Due to the way that class works, the first tick of this node
    will almost certainly leave it in the RUNNING state,
    even if the service responds very quickly.

    If this node is ticked again after having returned SUCCEEDED or
    FAILED, it will only call the service again
    when the service name or request data have been updated.
    """

    _service_type: type
    _service_client: Optional[Client] = None

    # Returns the service request message that should be send to the service.
    @abc.abstractmethod
    def get_request(self) -> Result[Any, BehaviorTreeException]:
        raise NotImplementedError

    # Property for the list of request fields that have to be monitored for updates
    @property
    @abc.abstractmethod
    def request_fields(self) -> list[str]:
        raise NotImplementedError

    # Sets the outputs relative to the given response.
    # Should return the desired node state as `Ok(...)` or an `Err(...)` with an exception.
    @abc.abstractmethod
    def set_outputs(self, response: Any) -> Result[BTNodeState, BehaviorTreeException]:
        raise NotImplementedError

    # Get the service type
    @abc.abstractmethod
    def get_service_type(self) -> Result[type, NodeConfigError]:
        raise NotImplementedError

    def _do_setup(self) -> Result[BTNodeState, BehaviorTreeException]:
        self._service_client: Optional[Client] = None
        self._service_request_future: Optional[Future] = None

        match self.inputs.get_value_as("wait_for_response_seconds", float):
            case Err(e):
                return Err(e)
            case Ok(f):
                self._timeout_seconds = f

        match self.get_service_type():
            case Err(e):
                return Err(e)
            case Ok(t):
                self._service_type = t

        if not self.has_ros_node:
            return Err(
                BehaviorTreeException(
                    "ROS service node does not have ROS node reference!"
                )
            )

        return Ok(BTNodeState.IDLE)

    def _do_tick(self) -> Result[BTNodeState, BehaviorTreeException]:
        # If the service name or request changed, discard and restart the request
        match self.inputs.any_updated("service_name", *self.request_fields):
            case Err(e):
                return Err(e)
            case Ok(b):
                updated = b
        if updated:
            match self.get_request():
                case Err(e):
                    return Err(e)
                case Ok(r):
                    self._input_request = r
            match self._do_reset():
                case Err(e):
                    return Err(e)
                case Ok(_):
                    pass

        if self._service_client is None:
            match self.inputs.get_value_as("service_name", str):
                case Err(e):
                    return Err(e)
                case Ok(v):
                    service_name = v
            self._service_client = self.ros_node.create_client(
                self._service_type,
                service_name,
                callback_group=ReentrantCallbackGroup(),
            )

        if self._input_request is not None:
            self._reported_result = False
            self._last_service_call_time = self.ros_node.get_clock().now()
            self._service_request_future = self._service_client.call_async(
                self._input_request
            )
            self._input_request = None

        if self._service_request_future is None:
            return Ok(self.state)

        if self._service_request_future.done():
            res = self._service_request_future.result()
            if res is None:
                return Err(BehaviorTreeException("Service response is none!"))
            self._service_request_future = None
            return self.set_outputs(res)
        if self._service_request_future.cancelled():
            self._service_request_future = None
            return Ok(BTNodeState.FAILED)

        # If the call takes longer than the specified timeout, abort the
        # call and return FAILED
        if self._last_service_call_time is None:
            self.logdebug("No previous timeout start timestamp set! Timeout starts now")
            self._last_service_call_time = self.ros_node.get_clock().now()

        seconds_since_call: float = (
            self.ros_node.get_clock().now() - self._last_service_call_time
        ).nanoseconds / 1e9

        if seconds_since_call > self._timeout_seconds:
            self.logwarn(
                f"Service call for node {self.name} timed out after {seconds_since_call} seconds"
            )
            self._service_request_future.cancel()
            self._service_request_future = None
            return Ok(BTNodeState.FAILED)

        return Ok(BTNodeState.RUNNING)

    def _do_untick(self) -> Result[BTNodeState, BehaviorTreeException]:
        if self._service_request_future is not None:
            self._service_request_future.cancel()
        self._service_request_future = None
        return Ok(BTNodeState.IDLE)

    def _do_reset(self) -> Result[BTNodeState, BehaviorTreeException]:
        if self._service_client is not None:
            if not self.ros_node.destroy_client(self._service_client):
                return Err(
                    BehaviorTreeException(
                        f"Failed to destroy service client in {self.name}"
                    )
                )
            self._service_client = None
        return self._do_untick()

    def _do_shutdown(self) -> Result[BTNodeState, BehaviorTreeException]:
        return self._do_reset().map(lambda _: BTNodeState.SHUTDOWN)

    def _do_calculate_utility(self) -> Result[UtilityBounds, BehaviorTreeException]:
        if not self.has_ros_node or self._service_client is None:
            self.logwarn(
                "Unable to check for service, no ros node or service available!"
            )
            return Ok(UtilityBounds())

        if self._service_client.service_is_ready():
            self.loginfo(
                "Found service with correct type, returning filled out UtilityBounds"
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

        self.logwarn("Service is unavailable")
        return Ok(UtilityBounds(can_execute=False))


@define_bt_node(
    NodeConfig(
        inputs={"service_type": RosServiceType(interface_id=1)},
        outputs={},
        max_children=0,
    )
)
class Service(ServiceBase):
    """
    A general implementation of a ROS service node,
    which has the service type as an input and exposes
    all request/response fields as additional inputs/outputs.
    """

    def get_request(self) -> Result[Any, BehaviorTreeException]:
        request = self._request_type()
        for key in self._request_fields:
            match self.inputs.get_value(key):
                case Err(e):
                    return Err(e)
                case Ok(v):
                    setattr(request, key, v)
        return Ok(request)

    @property
    def request_fields(self) -> list[str]:
        return self._request_fields

    def set_outputs(self, response: Any) -> Result[BTNodeState, BehaviorTreeException]:
        try:
            response_values = {
                key: getattr(response, key) for key in self._response_fields
            }
        except AttributeError as e:
            return Err(BehaviorTreeException(str(e)))
        return self.outputs.set_multiple_values(**response_values).map(
            lambda _: BTNodeState.SUCCEEDED
        )

    def get_service_type(self) -> Result[type, NodeConfigError]:
        return self.inputs.get_value_as("service_type", type)

    def add_extra_inputs(self) -> Result[dict[str, DataContainer], NodeConfigError]:
        match self.get_service_type():
            case Err(e):
                return Err(e)
            case Ok(t):
                service_type = t
        self._service_type = service_type
        self._request_type = service_type.Request
        inputs = {}
        for (
            field_name,
            field_type,
        ) in self._request_type.get_fields_and_field_types().items():
            match get_message_field_io_type(field_type):
                case Err(e):
                    return Err(NodeConfigError(e))
                case Ok(t):
                    inputs[field_name] = t
        return Ok(inputs)

    def add_extra_outputs(self) -> Result[dict[str, DataContainer], NodeConfigError]:
        match self.get_service_type():
            case Err(e):
                return Err(e)
            case Ok(t):
                service_type = t
        self._response_type = service_type.Response
        outputs = {}
        for (
            field_name,
            field_type,
        ) in self._response_type.get_fields_and_field_types().items():
            match get_message_field_io_type(field_type):
                case Err(e):
                    return Err(NodeConfigError(e))
                case Ok(t):
                    outputs[field_name] = t
        return Ok(outputs)

    def _do_setup(self) -> Result[BTNodeState, BehaviorTreeException]:
        self._request_fields = list(
            self._request_type.get_fields_and_field_types().keys()
        )
        self._response_fields = list(
            self._response_type.get_fields_and_field_types().keys()
        )
        return super()._do_setup()


@define_bt_node(
    NodeConfig(
        inputs={
            "service_name": RosServiceName(interface_id=1),
            "service_type": RosServiceType(interface_id=1),
            "wait_for_service_seconds": FloatType(allow_dynamic=False, value=10.2),
        },
        outputs={},
        max_children=0,
    )
)
class WaitForService(Leaf):
    """Wait for a service to be available, fails if this wait times out."""

    def _do_setup(self) -> Result[BTNodeState, BehaviorTreeException]:
        if not self.has_ros_node:
            self.logerr("No ROS node reference available!")
            return Err(BehaviorTreeException("No ROS node reference available!"))

        match self.inputs.get_value("service_type"):
            case Err(e):
                return Err(e)
            case Ok(t):
                self._service_type = t
        match self.inputs.get_value_as("wait_for_service_seconds", float):
            case Err(e):
                return Err(e)
            case Ok(f):
                self._wait_for_service_seconds = f

        self._service_client: Optional[Client] = None
        self._last_service_call_time: Optional[Time] = None
        return Ok(BTNodeState.IDLE)

    def _do_tick(self) -> Result[BTNodeState, BehaviorTreeException]:
        match self.inputs.any_updated("service_name"):
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

        if self._service_client is None:
            match self.inputs.get_value_as("service_name", str):
                case Err(e):
                    return Err(e)
                case Ok(s):
                    service_name = s
            self._service_client = self.ros_node.create_client(
                self._service_type,
                service_name,
            )

        if self._service_client.service_is_ready():
            return Ok(BTNodeState.SUCCEEDED)
        else:
            if self._last_service_call_time is None:
                self._last_service_call_time = self.ros_node.get_clock().now()
            elapsed_time: Duration = (
                self.ros_node.get_clock().now() - self._last_service_call_time
            )  # type: ignore   We know that Time - Time = Duration

            if (elapsed_time.nanoseconds / 1e9) > self._wait_for_service_seconds:
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
        return self._do_untick()

    def _do_shutdown(self) -> Result[BTNodeState, BehaviorTreeException]:
        return self._do_reset().map(lambda _: BTNodeState.SHUTDOWN)
