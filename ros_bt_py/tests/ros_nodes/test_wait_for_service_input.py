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
from ros_bt_py.custom_types import RosServiceType
from ros_bt_py.ros_nodes.service import WaitForServiceInput
from rclpy.time import Time
from ros_bt_py_interfaces.msg import NodeState
from ros_bt_py.exceptions import BehaviorTreeException


@mock.patch("rclpy.node.Node")
@mock.patch("rclpy.client.Client")
@mock.patch("rclpy.clock.Clock")
def test_node_success(ros_mock, client_mock, clock_mock):
    client_mock.service_is_ready.return_value = False
    ros_mock.create_client.return_value = client_mock
    clock_mock.now.side_effect = [Time(seconds=0), Time(seconds=1), Time(seconds=2)]
    ros_mock.get_clock.return_value = clock_mock

    unavailable_service = WaitForServiceInput(
        options={
            "service_type": RosServiceType("std_srvs/srv/SetBool"),
            "wait_for_service_seconds": 5.0,
        },
        ros_node=ros_mock,
    )

    assert unavailable_service is not None
    unavailable_service.setup()
    assert unavailable_service.state == NodeState.IDLE
    unavailable_service.inputs["service_name"] = "this_service_does_not_exist"
    unavailable_service.tick()
    assert unavailable_service.state == NodeState.RUNNING

    client_mock.service_is_ready.return_value = True

    unavailable_service.tick()
    assert unavailable_service.state == NodeState.SUCCEEDED

    unavailable_service.shutdown()
    assert unavailable_service.state == NodeState.SHUTDOWN
    assert ros_mock.destroy_client.called


@mock.patch("rclpy.node.Node")
@mock.patch("rclpy.client.Client")
@mock.patch("rclpy.clock.Clock")
def test_node_failure(ros_mock, client_mock, clock_mock):
    client_mock.service_is_ready.return_value = False
    ros_mock.create_client.return_value = client_mock
    clock_mock.now.side_effect = [Time(seconds=0), Time(seconds=1), Time(seconds=10)]
    ros_mock.get_clock.return_value = clock_mock

    unavailable_service = WaitForServiceInput(
        options={
            "service_type": RosServiceType("std_srvs/srv/SetBool"),
            "wait_for_service_seconds": 5.0,
        },
        ros_node=ros_mock,
    )

    assert unavailable_service is not None
    unavailable_service.setup()
    assert unavailable_service.state == NodeState.IDLE
    unavailable_service.inputs["service_name"] = "this_service_does_not_exist"
    unavailable_service.tick()
    assert unavailable_service.state == NodeState.RUNNING

    unavailable_service.tick()
    assert unavailable_service.state == NodeState.FAILED

    unavailable_service.shutdown()
    assert unavailable_service.state == NodeState.SHUTDOWN
    assert ros_mock.destroy_client.called


@mock.patch("rclpy.node.Node")
@mock.patch("rclpy.client.Client")
@mock.patch("rclpy.clock.Clock")
def test_node_reset(ros_mock, client_mock, clock_mock):
    client_mock.service_is_ready.return_value = False
    ros_mock.create_client.return_value = client_mock
    clock_mock.now.side_effect = [
        Time(seconds=0),
        Time(seconds=1),
        Time(seconds=10),
        Time(seconds=11),
        Time(seconds=12),
    ]
    ros_mock.get_clock.return_value = clock_mock

    unavailable_service = WaitForServiceInput(
        options={
            "service_type": RosServiceType("std_srvs/srv/SetBool"),
            "wait_for_service_seconds": 5.0,
        },
        ros_node=ros_mock,
    )

    assert unavailable_service is not None
    unavailable_service.setup()
    assert unavailable_service.state == NodeState.IDLE
    unavailable_service.inputs["service_name"] = "this_service_does_not_exist"
    unavailable_service.tick()
    assert unavailable_service.state == NodeState.RUNNING
    unavailable_service.reset()
    assert unavailable_service._last_service_call_time is None  # type: ignore

    unavailable_service.tick()
    assert unavailable_service.state == NodeState.RUNNING

    unavailable_service.tick()
    assert unavailable_service.state == NodeState.RUNNING

    unavailable_service.shutdown()
    assert unavailable_service.state == NodeState.SHUTDOWN
    assert ros_mock.destroy_client.called


def test_node_no_ros():
    unavailable_service = WaitForServiceInput(
        options={
            "service_type": RosServiceType("std_srvs/srv/SetBool"),
            "wait_for_service_seconds": 5.0,
        },
        ros_node=None,
    )

    assert unavailable_service is not None
    setup_result = unavailable_service.setup()
    assert isinstance(setup_result.err(), BehaviorTreeException)


@mock.patch("rclpy.node.Node")
@mock.patch("rclpy.client.Client")
@mock.patch("rclpy.clock.Clock")
def test_node_untick(ros_mock, client_mock, clock_mock):
    client_mock.service_is_ready.return_value = False
    ros_mock.create_client.return_value = client_mock
    clock_mock.now.side_effect = [
        Time(seconds=0),
        Time(seconds=1),
        Time(seconds=10),
        Time(seconds=11),
        Time(seconds=12),
    ]
    ros_mock.get_clock.return_value = clock_mock

    unavailable_service = WaitForServiceInput(
        options={
            "service_type": RosServiceType("std_srvs/srv/SetBool"),
            "wait_for_service_seconds": 5.0,
        },
        ros_node=ros_mock,
    )

    assert unavailable_service is not None
    unavailable_service.setup()
    assert unavailable_service.state == NodeState.IDLE
    unavailable_service.inputs["service_name"] = "this_service_does_not_exist"
    unavailable_service.tick()
    assert unavailable_service.state == NodeState.RUNNING
    unavailable_service.untick()
    assert unavailable_service._last_service_call_time is None  # type: ignore

    unavailable_service.tick()
    assert unavailable_service.state == NodeState.RUNNING

    unavailable_service.tick()
    assert unavailable_service.state == NodeState.RUNNING

    unavailable_service.shutdown()
    assert unavailable_service.state == NodeState.SHUTDOWN
    assert ros_mock.destroy_client.called
