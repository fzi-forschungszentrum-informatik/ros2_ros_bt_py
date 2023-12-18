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
from unittest.mock import patch
from rclpy.time import Time
from ros_bt_py.exceptions import BehaviorTreeException
from ros_bt_py.ros_nodes.param import RosParamOptionDefaultInput
from ros_bt_py_interfaces.msg import Node as NodeMsg, UtilityBounds

import pytest


def has_parameter_mock(name: str) -> bool:
    if name == "this_service_does_not_exist":
        return True
    else:
        return False


@patch("rclpy.node.Node")
@patch("rclpy.clock.Clock")
@patch("rclpy.parameter.Parameter")
def test_node_success_none(ros_mock, clock_mock, parameter_mock):
    clock_mock.now.side_effect = [Time(seconds=0), Time(seconds=1), Time(seconds=2)]
    ros_mock.get_clock.return_value = clock_mock
    ros_mock.has_parameter.side_effect = has_parameter_mock
    parameter_mock.value = None
    ros_mock.get_parameter.return_value = parameter_mock

    param_node = RosParamOptionDefaultInput(
        options={
            "param_name": "this_service_does_not_exist",
            "param_type": str,
        },
        ros_node=ros_mock,
    )
    param_node.setup()
    assert param_node.state == NodeMsg.IDLE
    param_node.inputs["default_value"] = "tests"
    param_node.tick()
    assert param_node.state == NodeMsg.SUCCEEDED
    assert param_node.outputs["param"] == "tests"


@patch("rclpy.node.Node")
@patch("rclpy.clock.Clock")
@patch("rclpy.parameter.Parameter")
def test_node_success(ros_mock, clock_mock, parameter_mock):
    clock_mock.now.side_effect = [Time(seconds=0), Time(seconds=1), Time(seconds=2)]
    ros_mock.get_clock.return_value = clock_mock
    ros_mock.has_parameter.side_effect = has_parameter_mock
    parameter_mock.value = str("not_tests")
    ros_mock.get_parameter.return_value = parameter_mock

    param_node = RosParamOptionDefaultInput(
        options={
            "param_name": "this_service_does_not_exist",
            "param_type": str,
        },
        ros_node=ros_mock,
    )
    param_node.setup()
    assert param_node.state == NodeMsg.IDLE
    param_node.inputs["default_value"] = "tests"
    param_node.tick()
    assert param_node.state == NodeMsg.SUCCEEDED
    assert param_node.outputs["param"] == "not_tests"

    param_node.shutdown()
    assert param_node.state == NodeMsg.SHUTDOWN


@patch("rclpy.node.Node")
@patch("rclpy.clock.Clock")
def test_node_failure(ros_mock, clock_mock):
    clock_mock.now.side_effect = [Time(seconds=0), Time(seconds=1), Time(seconds=2)]
    ros_mock.get_clock.return_value = clock_mock
    ros_mock.has_parameter.side_effect = has_parameter_mock

    param_node = RosParamOptionDefaultInput(
        options={
            "param_name": "bla",
            "param_type": str,
        },
        ros_node=ros_mock,
    )
    param_node.setup()
    assert param_node.state == NodeMsg.IDLE
    param_node.inputs["default_value"] = "tests"
    param_node.tick()
    assert param_node.state == NodeMsg.FAILURE

    param_node.shutdown()
    assert param_node.state == NodeMsg.SHUTDOWN


@patch("rclpy.node.Node")
@patch("rclpy.clock.Clock")
@patch("rclpy.parameter.Parameter")
def test_node_failure_invalid_type(ros_mock, clock_mock, parameter_mock):
    clock_mock.now.side_effect = [Time(seconds=0), Time(seconds=1), Time(seconds=2)]
    ros_mock.get_clock.return_value = clock_mock
    ros_mock.has_parameter.side_effect = has_parameter_mock
    parameter_mock.value = 1
    ros_mock.get_parameter.return_value = parameter_mock

    param_node = RosParamOptionDefaultInput(
        options={
            "param_name": "this_service_does_not_exist",
            "param_type": str,
        },
        ros_node=ros_mock,
    )
    param_node.setup()
    assert param_node.state == NodeMsg.IDLE
    param_node.inputs["default_value"] = "tests"
    param_node.tick()
    assert param_node.state == NodeMsg.FAILURE


@patch("rclpy.node.Node")
@patch("rclpy.clock.Clock")
@patch("rclpy.parameter.Parameter")
def test_node_reset(ros_mock, clock_mock, parameter_mock):
    clock_mock.now.side_effect = [Time(seconds=0), Time(seconds=1), Time(seconds=2)]
    ros_mock.get_clock.return_value = clock_mock
    ros_mock.has_parameter.side_effect = has_parameter_mock
    parameter_mock.value = str("not_tests")
    ros_mock.get_parameter.return_value = parameter_mock

    param_node = RosParamOptionDefaultInput(
        options={
            "param_name": "this_service_does_not_exist",
            "param_type": str,
        },
        ros_node=ros_mock,
    )
    param_node.setup()
    assert param_node.state == NodeMsg.IDLE
    param_node.inputs["default_value"] = "tests"
    param_node.tick()
    assert param_node.state == NodeMsg.SUCCEEDED
    assert param_node.outputs["param"] == "not_tests"
    param_node.reset()
    assert param_node.state == NodeMsg.IDLE

    param_node.tick()
    assert param_node.state == NodeMsg.SUCCEEDED
    assert param_node.outputs["param"] == "not_tests"

    param_node.shutdown()
    assert param_node.state == NodeMsg.SHUTDOWN


@patch("rclpy.node.Node")
@patch("rclpy.clock.Clock")
@patch("rclpy.parameter.Parameter")
def test_node_untick(ros_mock, clock_mock, parameter_mock):
    clock_mock.now.side_effect = [Time(seconds=0), Time(seconds=1), Time(seconds=2)]
    ros_mock.get_clock.return_value = clock_mock
    ros_mock.has_parameter.side_effect = has_parameter_mock
    parameter_mock.value = str("not_tests")
    ros_mock.get_parameter.return_value = parameter_mock

    param_node = RosParamOptionDefaultInput(
        options={
            "param_name": "this_service_does_not_exist",
            "param_type": str,
        },
        ros_node=ros_mock,
    )
    param_node.setup()
    assert param_node.state == NodeMsg.IDLE
    param_node.inputs["default_value"] = "tests"
    param_node.tick()
    assert param_node.state == NodeMsg.SUCCEEDED
    assert param_node.outputs["param"] == "not_tests"
    param_node.untick()
    assert param_node.state == NodeMsg.IDLE

    param_node.tick()
    assert param_node.state == NodeMsg.SUCCEEDED
    assert param_node.outputs["param"] == "not_tests"

    param_node.shutdown()
    assert param_node.state == NodeMsg.SHUTDOWN


def test_node_no_ros():

    param_node = RosParamOptionDefaultInput(
        options={
            "param_name": "this_service_does_not_exist",
            "param_type": str,
        },
        ros_node=None,
    )
    with pytest.raises(BehaviorTreeException):
        param_node.setup()


def test_node_utility():

    param_node = RosParamOptionDefaultInput(
        options={
            "param_name": "this_service_does_not_exist",
            "param_type": str,
        },
        ros_node=None,
    )
    utility = param_node.calculate_utility()
    assert utility == UtilityBounds()
