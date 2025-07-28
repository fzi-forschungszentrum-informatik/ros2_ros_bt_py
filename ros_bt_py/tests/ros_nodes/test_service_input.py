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
import pytest

import unittest.mock as mock
from std_srvs.srv import SetBool
from ros_bt_py.custom_types import RosServiceType
from ros_bt_py.ros_nodes.service import ServiceInput
from rclpy.time import Time
from ros_bt_py_interfaces.msg import NodeState, UtilityBounds
from ros_bt_py.exceptions import BehaviorTreeException


class TestServiceInput:
    @mock.patch("rclpy.node.Node")
    @mock.patch("rclpy.client.Client")
    @mock.patch("rclpy.client.Future")
    @mock.patch("rclpy.clock.Clock")
    def test_node_success(self, ros_mock, client_mock, future_mock, clock_mock):
        response = SetBool.Response()
        response.success = True
        future_mock.result.return_value = response
        future_mock.done.return_value = False
        future_mock.cancelled.return_value = False
        client_mock.call_async.return_value = future_mock
        ros_mock.create_client.return_value = client_mock
        clock_mock.now.side_effect = [Time(seconds=0), Time(seconds=1), Time(seconds=2)]
        ros_mock.get_clock.return_value = clock_mock
        unavailable_service = ServiceInput(
            options={
                "service_type": RosServiceType("std_srvs/srv/SetBool"),
                "wait_for_response_seconds": 5.0,
            },
            ros_node=ros_mock,
        )

        assert unavailable_service is not None
        unavailable_service.setup()
        assert unavailable_service.state == NodeState.IDLE
        unavailable_service.inputs["service_name"] = "this_service_does_not_exist"
        unavailable_service.inputs["data"] = SetBool.Request().data
        unavailable_service.tick()
        assert unavailable_service.state == NodeState.RUNNING

        future_mock.done.return_value = True

        unavailable_service.tick()
        assert unavailable_service.state == NodeState.SUCCEEDED
        assert unavailable_service.outputs["success"]

        unavailable_service.shutdown()
        assert unavailable_service.state == NodeState.SHUTDOWN
        assert ros_mock.destroy_client.called

    @mock.patch("rclpy.node.Node")
    @mock.patch("rclpy.client.Client")
    @mock.patch("rclpy.client.Future")
    @mock.patch("rclpy.clock.Clock")
    def test_node_failure(self, ros_mock, client_mock, future_mock, clock_mock):
        response = SetBool.Response()
        response.success = True
        future_mock.result.return_value = response
        future_mock.done.return_value = False
        future_mock.cancelled.return_value = True
        client_mock.call_async.return_value = future_mock
        ros_mock.create_client.return_value = client_mock
        clock_mock.now.side_effect = [Time(seconds=0), Time(seconds=1), Time(seconds=2)]
        ros_mock.get_clock.return_value = clock_mock

        unavailable_service = ServiceInput(
            options={
                "service_type": RosServiceType("std_srvs/srv/SetBool"),
                "wait_for_response_seconds": 5.0,
            },
            ros_node=ros_mock,
        )

        assert unavailable_service is not None
        unavailable_service.setup()
        assert unavailable_service.state == NodeState.IDLE
        unavailable_service.inputs["service_name"] = "this_service_does_not_exist"
        unavailable_service.inputs["data"] = SetBool.Request().data
        unavailable_service.tick()
        assert unavailable_service.state == NodeState.FAILED

    @mock.patch("rclpy.node.Node")
    @mock.patch("rclpy.client.Client")
    @mock.patch("rclpy.client.Future")
    @mock.patch("rclpy.clock.Clock")
    def test_node_timeout(self, ros_mock, client_mock, future_mock, clock_mock):
        response = SetBool.Response()
        response.success = True
        future_mock.result.return_value = response
        future_mock.done.return_value = False
        future_mock.cancelled.return_value = False
        client_mock.call_async.return_value = future_mock
        ros_mock.create_client.return_value = client_mock
        clock_mock.now.return_value = Time(seconds=0)
        ros_mock.get_clock.return_value = clock_mock

        unavailable_service = ServiceInput(
            options={
                "service_type": RosServiceType("std_srvs/srv/SetBool"),
                "wait_for_response_seconds": 5.0,
            },
            ros_node=ros_mock,
        )

        assert unavailable_service is not None
        unavailable_service.setup()
        assert unavailable_service.state == NodeState.IDLE
        unavailable_service.inputs["service_name"] = "this_service_does_not_exist"
        unavailable_service.inputs["data"] = SetBool.Request().data
        unavailable_service.tick()
        assert unavailable_service.state == NodeState.RUNNING

        clock_mock.now.return_value = Time(seconds=10)
        unavailable_service.tick()
        assert unavailable_service.state == NodeState.FAILED

    @mock.patch("rclpy.node.Node")
    @mock.patch("rclpy.client.Client")
    @mock.patch("rclpy.client.Future")
    @mock.patch("rclpy.clock.Clock")
    def test_node_reset_shutdown(self, ros_mock, client_mock, future_mock, clock_mock):
        response = SetBool.Response()
        response.success = True
        future_mock.result.return_value = response
        future_mock.done.return_value = False
        future_mock.cancelled.return_value = False
        client_mock.call_async.return_value = future_mock
        ros_mock.create_client.return_value = client_mock
        clock_mock.now.side_effect = [
            Time(seconds=0),
            Time(seconds=1),
            Time(seconds=2),
            Time(seconds=3),
            Time(seconds=4),
        ]
        ros_mock.get_clock.return_value = clock_mock

        unavailable_service = ServiceInput(
            options={
                "service_type": RosServiceType("std_srvs/srv/SetBool"),
                "wait_for_response_seconds": 5.0,
            },
            ros_node=ros_mock,
        )

        assert unavailable_service is not None
        unavailable_service.setup()
        assert unavailable_service.state == NodeState.IDLE
        unavailable_service.inputs["service_name"] = "this_service_does_not_exist"
        unavailable_service.inputs["data"] = SetBool.Request().data
        unavailable_service.tick()
        assert unavailable_service.state == NodeState.RUNNING

        future_mock.done.return_value = True

        unavailable_service.tick()
        assert unavailable_service.state == NodeState.SUCCEEDED
        assert unavailable_service.outputs["success"]

        unavailable_service.reset()
        assert unavailable_service.state == NodeState.IDLE
        assert ros_mock.destroy_client.called
        assert ros_mock.destroy_client.call_count == 1

        unavailable_service.shutdown()
        assert unavailable_service.state == NodeState.SHUTDOWN
        assert ros_mock.destroy_client.called
        assert ros_mock.destroy_client.call_count == 1

    @mock.patch("rclpy.node.Node")
    @mock.patch("rclpy.client.Client")
    @mock.patch("rclpy.client.Future")
    @mock.patch("rclpy.clock.Clock")
    def test_node_reset(self, ros_mock, client_mock, future_mock, clock_mock):
        response = SetBool.Response()
        response.success = True
        future_mock.result.return_value = response
        future_mock.done.return_value = False
        future_mock.cancelled.return_value = False
        client_mock.call_async.return_value = future_mock
        ros_mock.create_client.return_value = client_mock
        clock_mock.now.side_effect = [
            Time(seconds=0),
            Time(seconds=1),
            Time(seconds=2),
            Time(seconds=3),
            Time(seconds=4),
        ]
        ros_mock.get_clock.return_value = clock_mock

        unavailable_service = ServiceInput(
            options={
                "service_type": RosServiceType("std_srvs/srv/SetBool"),
                "wait_for_response_seconds": 5.0,
            },
            ros_node=ros_mock,
        )

        assert unavailable_service is not None
        unavailable_service.setup()
        assert unavailable_service.state == NodeState.IDLE
        unavailable_service.inputs["service_name"] = "this_service_does_not_exist"
        unavailable_service.inputs["data"] = SetBool.Request().data
        unavailable_service.tick()
        assert unavailable_service.state == NodeState.RUNNING

        future_mock.done.return_value = True

        unavailable_service.tick()
        assert unavailable_service.state == NodeState.SUCCEEDED
        assert unavailable_service.outputs["success"]

        unavailable_service.reset()

        future_mock.done.return_value = False
        response = SetBool.Response()
        response.success = False
        future_mock.result.return_value = response

        clock_mock.now.side_effect = [
            Time(seconds=0),
            Time(seconds=1),
            Time(seconds=2),
            Time(seconds=3),
            Time(seconds=4),
        ]

        assert unavailable_service.state == NodeState.IDLE
        unavailable_service.inputs["service_name"] = "this_service_does_not_exist"
        unavailable_service.inputs["data"] = SetBool.Request().data
        unavailable_service.tick()
        assert unavailable_service.state == NodeState.RUNNING

        future_mock.done.return_value = True

        unavailable_service.tick()
        assert unavailable_service.state == NodeState.SUCCEEDED
        assert not unavailable_service.outputs["success"]

    def test_node_no_ros(self):
        unavailable_service = ServiceInput(
            options={
                "service_type": RosServiceType("std_srvs/srv/SetBool"),
                "wait_for_response_seconds": 5.0,
            },
            ros_node=None,
        )

        assert unavailable_service is not None
        setup_result = unavailable_service.setup()
        assert isinstance(setup_result.err(), BehaviorTreeException)

    @mock.patch("rclpy.node.Node")
    @mock.patch("rclpy.client.Client")
    @mock.patch("rclpy.client.Future")
    @mock.patch("rclpy.clock.Clock")
    def test_node_name_change(self, ros_mock, client_mock, future_mock, clock_mock):
        response = SetBool.Response()
        response.success = True
        future_mock.result.return_value = response
        future_mock.done.return_value = False
        future_mock.cancelled.return_value = False
        client_mock.call_async.return_value = future_mock
        ros_mock.create_client.return_value = client_mock
        clock_mock.now.side_effect = [
            Time(seconds=0),
            Time(seconds=1),
            Time(seconds=2),
            Time(seconds=3),
            Time(seconds=4),
        ]
        ros_mock.get_clock.return_value = clock_mock

        unavailable_service = ServiceInput(
            options={
                "service_type": RosServiceType("std_srvs/srv/SetBool"),
                "wait_for_response_seconds": 5.0,
            },
            ros_node=ros_mock,
        )

        assert unavailable_service is not None
        unavailable_service.setup()
        assert unavailable_service.state == NodeState.IDLE
        unavailable_service.inputs["service_name"] = "this_service_does_not_exist"
        unavailable_service.inputs["data"] = SetBool.Request().data
        unavailable_service.tick()
        assert unavailable_service.state == NodeState.RUNNING

        unavailable_service.inputs["service_name"] = "bla"
        unavailable_service.tick()
        assert unavailable_service.state == NodeState.RUNNING
        assert ros_mock.destroy_client.called

    @mock.patch("rclpy.node.Node")
    @mock.patch("rclpy.client.Client")
    @mock.patch("rclpy.client.Future")
    @mock.patch("rclpy.clock.Clock")
    def test_node_untick(self, ros_mock, client_mock, future_mock, clock_mock):
        response = SetBool.Response()
        response.success = True
        future_mock.result.return_value = response
        future_mock.done.return_value = False
        future_mock.cancelled.return_value = False
        client_mock.call_async.return_value = future_mock
        ros_mock.create_client.return_value = client_mock
        clock_mock.now.side_effect = [
            Time(seconds=0),
            Time(seconds=1),
            Time(seconds=2),
            Time(seconds=3),
            Time(seconds=4),
        ]
        ros_mock.get_clock.return_value = clock_mock

        unavailable_service = ServiceInput(
            options={
                "service_type": RosServiceType("std_srvs/srv/SetBool"),
                "wait_for_response_seconds": 5.0,
            },
            ros_node=ros_mock,
        )

        assert unavailable_service is not None
        unavailable_service.setup()
        assert unavailable_service.state == NodeState.IDLE
        unavailable_service.inputs["service_name"] = "this_service_does_not_exist"
        unavailable_service.inputs["data"] = SetBool.Request().data
        unavailable_service.tick()
        assert unavailable_service.state == NodeState.RUNNING

        unavailable_service.untick()
        assert unavailable_service.state == NodeState.IDLE
        assert future_mock.cancel.called

    @mock.patch("rclpy.node.Node")
    @mock.patch("rclpy.client.Client")
    @mock.patch("rclpy.client.Future")
    @mock.patch("rclpy.clock.Clock")
    def test_node_utility_no_ros(self, ros_mock, client_mock, future_mock, clock_mock):
        response = SetBool.Response()
        response.success = True
        future_mock.result.return_value = response
        future_mock.done.return_value = False
        future_mock.cancelled.return_value = False
        client_mock.call_async.return_value = future_mock
        ros_mock.create_client.return_value = client_mock
        clock_mock.now.side_effect = [
            Time(seconds=0),
            Time(seconds=1),
            Time(seconds=2),
            Time(seconds=3),
            Time(seconds=4),
        ]
        ros_mock.get_clock.return_value = clock_mock

        unavailable_service = ServiceInput(
            options={
                "service_type": RosServiceType("std_srvs/srv/SetBool"),
                "wait_for_response_seconds": 5.0,
            },
            ros_node=None,
        )
        bounds_result = unavailable_service.calculate_utility()
        assert bounds_result.ok() == UtilityBounds()

        unavailable_service_2 = ServiceInput(
            options={
                "service_type": RosServiceType("std_srvs/srv/SetBool"),
                "wait_for_response_seconds": 5.0,
            },
            ros_node=ros_mock,
        )
        assert unavailable_service_2 is not None
        unavailable_service_2.setup()
        assert unavailable_service_2.state == NodeState.IDLE
        bounds_result_2 = unavailable_service_2.calculate_utility()
        assert bounds_result_2.ok() == UtilityBounds()

    @mock.patch("rclpy.node.Node")
    @mock.patch("rclpy.client.Client")
    @mock.patch("rclpy.client.Future")
    @mock.patch("rclpy.clock.Clock")
    def test_node_utility(self, ros_mock, client_mock, future_mock, clock_mock):
        response = SetBool.Response()
        response.success = True
        future_mock.result.return_value = response
        future_mock.done.return_value = False
        future_mock.cancelled.return_value = False
        client_mock.call_async.return_value = future_mock
        ros_mock.create_client.return_value = client_mock
        clock_mock.now.side_effect = [
            Time(seconds=0),
            Time(seconds=1),
            Time(seconds=2),
            Time(seconds=3),
            Time(seconds=4),
        ]
        ros_mock.get_clock.return_value = clock_mock

        unavailable_service = ServiceInput(
            options={
                "service_type": RosServiceType("std_srvs/srv/SetBool"),
                "wait_for_response_seconds": 5.0,
            },
            ros_node=ros_mock,
        )
        assert unavailable_service is not None
        unavailable_service.setup()
        assert unavailable_service.state == NodeState.IDLE
        unavailable_service.inputs["service_name"] = "this_service_does_not_exist"
        unavailable_service.inputs["data"] = SetBool.Request().data
        unavailable_service.tick()
        assert unavailable_service.state == NodeState.RUNNING

        client_mock.service_is_ready.return_value = False
        bounds_result = unavailable_service.calculate_utility()
        assert bounds_result.ok() == UtilityBounds(can_execute=False)

        client_mock.service_is_ready.return_value = True
        bounds_result = unavailable_service.calculate_utility()
        assert bounds_result.ok() == UtilityBounds(
            can_execute=True,
            has_lower_bound_success=True,
            has_upper_bound_success=True,
            has_lower_bound_failure=True,
            has_upper_bound_failure=True,
        )
