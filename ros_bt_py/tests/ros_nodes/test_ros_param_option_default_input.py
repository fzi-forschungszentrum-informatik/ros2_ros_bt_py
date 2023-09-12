from unittest.mock import patch
from rclpy.time import Time
from ros_bt_py.exceptions import BehaviorTreeException
from ros_bt_py.ros_nodes.param import RosParamOptionDefaultInput
from ros_bt_py_interfaces.msg import Node as NodeMsg, UtilityBounds

import pytest

from rclpy.parameter import Parameter


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
