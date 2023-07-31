import pytest

import unittest.mock as mock
from rclpy.client import Future
from std_srvs.srv import SetBool
from ros_bt_py.ros_nodes.service import Service
from rclpy.time import Time
from ros_bt_py_interfaces.msg import Node as NodeMsg, UtilityBounds
from ros_bt_py.exceptions import BehaviorTreeException


@mock.patch("rclpy.node.Node")
@mock.patch("rclpy.client.Client")
@mock.patch("rclpy.client.Future")
@mock.patch("rclpy.clock.Clock")
def test_node_success(ros_mock, client_mock, future_mock, clock_mock):
    response = SetBool.Response()
    response.success = True
    future_mock.result.return_value = response
    future_mock.done.return_value = False
    future_mock.cancelled.return_value = False
    client_mock.call_async.return_value = future_mock
    ros_mock.create_client.return_value = client_mock
    clock_mock.now.side_effect = [Time(seconds=0), Time(seconds=1), Time(seconds=2)]
    ros_mock.get_clock.return_value = clock_mock

    unavailable_service = Service(
        options={
            "service_name": "this_service_does_not_exist",
            "service_type": SetBool,
            "request_type": SetBool.Request,
            "response_type": SetBool.Response,
            "wait_for_response_seconds": 5.0,
            "wait_for_service_seconds": 5.0,
            "fail_if_not_available": True,
        },
        ros_node=ros_mock,
    )

    assert unavailable_service is not None
    unavailable_service.setup()
    assert unavailable_service.state == NodeMsg.IDLE
    request = SetBool.Request()
    request.data = False
    unavailable_service.inputs["request"] = request
    unavailable_service.tick()
    assert unavailable_service.state == NodeMsg.RUNNING
    assert client_mock.call_async.called_with(request)

    future_mock.done.return_value = True

    unavailable_service.tick()
    assert unavailable_service.state == NodeMsg.SUCCEEDED
    assert unavailable_service.outputs["response"].success

    unavailable_service.shutdown()
    assert unavailable_service.state == NodeMsg.SHUTDOWN
    assert client_mock.destroy.called


@mock.patch("rclpy.node.Node")
@mock.patch("rclpy.client.Client")
@mock.patch("rclpy.client.Future")
@mock.patch("rclpy.clock.Clock")
def test_node_failure(ros_mock, client_mock, future_mock, clock_mock):
    response = SetBool.Response()
    response.success = True
    future_mock.result.return_value = response
    future_mock.done.return_value = False
    future_mock.cancelled.return_value = True
    client_mock.call_async.return_value = future_mock
    ros_mock.create_client.return_value = client_mock
    clock_mock.now.side_effect = [Time(seconds=0), Time(seconds=1), Time(seconds=2)]
    ros_mock.get_clock.return_value = clock_mock

    unavailable_service = Service(
        options={
            "service_name": "this_service_does_not_exist",
            "service_type": SetBool,
            "request_type": SetBool.Request,
            "response_type": SetBool.Response,
            "wait_for_response_seconds": 5.0,
            "wait_for_service_seconds": 5.0,
            "fail_if_not_available": True,
        },
        ros_node=ros_mock,
    )

    assert unavailable_service is not None
    unavailable_service.setup()
    assert unavailable_service.state == NodeMsg.IDLE
    unavailable_service.inputs["request"] = SetBool.Request()
    unavailable_service.tick()
    assert unavailable_service.state == NodeMsg.FAILED


@mock.patch("rclpy.node.Node")
@mock.patch("rclpy.client.Client")
@mock.patch("rclpy.client.Future")
@mock.patch("rclpy.clock.Clock")
def test_node_timeout(ros_mock, client_mock, future_mock, clock_mock):
    response = SetBool.Response()
    response.success = True
    future_mock.result.return_value = response
    future_mock.done.return_value = False
    future_mock.cancelled.return_value = False
    client_mock.call_async.return_value = future_mock
    ros_mock.create_client.return_value = client_mock
    clock_mock.now.return_value = Time(seconds=0)
    ros_mock.get_clock.return_value = clock_mock

    unavailable_service = Service(
        options={
            "service_name": "this_service_does_not_exist",
            "service_type": SetBool,
            "request_type": SetBool.Request,
            "response_type": SetBool.Response,
            "wait_for_response_seconds": 5.0,
            "wait_for_service_seconds": 5.0,
            "fail_if_not_available": True,
        },
        ros_node=ros_mock,
    )

    assert unavailable_service is not None
    unavailable_service.setup()
    assert unavailable_service.state == NodeMsg.IDLE
    unavailable_service.inputs["request"] = SetBool.Request()
    unavailable_service.tick()
    assert unavailable_service.state == NodeMsg.RUNNING

    clock_mock.now.return_value = Time(seconds=10)
    unavailable_service.tick()
    assert unavailable_service.state == NodeMsg.FAILED


@mock.patch("rclpy.node.Node")
@mock.patch("rclpy.client.Client")
@mock.patch("rclpy.client.Future")
@mock.patch("rclpy.clock.Clock")
def test_node_reset_shutdown(ros_mock, client_mock, future_mock, clock_mock):
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

    unavailable_service = Service(
        options={
            "service_name": "this_service_does_not_exist",
            "service_type": SetBool,
            "request_type": SetBool.Request,
            "response_type": SetBool.Response,
            "wait_for_response_seconds": 5.0,
            "wait_for_service_seconds": 5.0,
            "fail_if_not_available": True,
        },
        ros_node=ros_mock,
    )

    assert unavailable_service is not None
    unavailable_service.setup()
    assert unavailable_service.state == NodeMsg.IDLE
    unavailable_service.inputs["request"] = SetBool.Request()
    unavailable_service.tick()
    assert unavailable_service.state == NodeMsg.RUNNING

    future_mock.done.return_value = True

    unavailable_service.tick()
    assert unavailable_service.state == NodeMsg.SUCCEEDED
    assert unavailable_service.outputs["response"].success

    unavailable_service.reset()
    assert unavailable_service.state == NodeMsg.IDLE

    unavailable_service.shutdown()
    assert unavailable_service.state == NodeMsg.SHUTDOWN
    assert client_mock.destroy.called
    assert client_mock.destroy.call_count == 1


@mock.patch("rclpy.node.Node")
@mock.patch("rclpy.client.Client")
@mock.patch("rclpy.client.Future")
@mock.patch("rclpy.clock.Clock")
def test_node_reset(ros_mock, client_mock, future_mock, clock_mock):
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

    unavailable_service = Service(
        options={
            "service_name": "this_service_does_not_exist",
            "service_type": SetBool,
            "request_type": SetBool.Request,
            "response_type": SetBool.Response,
            "wait_for_response_seconds": 5.0,
            "wait_for_service_seconds": 5.0,
            "fail_if_not_available": True,
        },
        ros_node=ros_mock,
    )

    assert unavailable_service is not None
    unavailable_service.setup()
    assert unavailable_service.state == NodeMsg.IDLE
    unavailable_service.inputs["request"] = SetBool.Request()
    unavailable_service.tick()
    assert unavailable_service.state == NodeMsg.RUNNING

    future_mock.done.return_value = True

    unavailable_service.tick()
    assert unavailable_service.state == NodeMsg.SUCCEEDED
    assert unavailable_service.outputs["response"].success

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

    assert unavailable_service.state == NodeMsg.IDLE
    unavailable_service.inputs["request"] = SetBool.Request()
    unavailable_service.tick()
    assert unavailable_service.state == NodeMsg.RUNNING

    future_mock.done.return_value = True

    unavailable_service.tick()
    assert unavailable_service.state == NodeMsg.SUCCEEDED
    assert not unavailable_service.outputs["response"].success


def test_node_no_ros():
    with pytest.raises(BehaviorTreeException):
        unavailable_service = Service(
            options={
                "service_name": "this_service_does_not_exist",
                "service_type": SetBool,
                "request_type": SetBool.Request,
                "response_type": SetBool.Response,
                "wait_for_response_seconds": 5.0,
                "wait_for_service_seconds": 5.0,
                "fail_if_not_available": True,
            },
            ros_node=None,
        )

        assert unavailable_service is not None
        unavailable_service.setup()


@mock.patch("rclpy.node.Node")
@mock.patch("rclpy.client.Client")
@mock.patch("rclpy.client.Future")
@mock.patch("rclpy.clock.Clock")
def test_node_untick(ros_mock, client_mock, future_mock, clock_mock):
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

    unavailable_service = Service(
        options={
            "service_name": "this_service_does_not_exist",
            "service_type": SetBool,
            "request_type": SetBool.Request,
            "response_type": SetBool.Response,
            "wait_for_response_seconds": 5.0,
            "wait_for_service_seconds": 5.0,
            "fail_if_not_available": True,
        },
        ros_node=ros_mock,
    )

    assert unavailable_service is not None
    unavailable_service.setup()
    assert unavailable_service.state == NodeMsg.IDLE
    unavailable_service.inputs["request"] = SetBool.Request()
    unavailable_service.tick()
    assert unavailable_service.state == NodeMsg.RUNNING

    unavailable_service.untick()
    assert unavailable_service.state == NodeMsg.IDLE
    assert future_mock.cancel.called


@mock.patch("rclpy.node.Node")
@mock.patch("rclpy.client.Client")
@mock.patch("rclpy.client.Future")
@mock.patch("rclpy.clock.Clock")
def test_node_utility_no_ros(ros_mock, client_mock, future_mock, clock_mock):
    response = SetBool.Response()
    response.success = True
    future_mock.result.return_value = response
    future_mock.done.return_value = False
    future_mock.cancelled.return_value = False
    client_mock.call_async.return_value = future_mock
    client_mock.service_is_ready.return_value = True
    ros_mock.create_client.return_value = client_mock
    clock_mock.now.side_effect = [
        Time(seconds=0),
        Time(seconds=1),
        Time(seconds=2),
        Time(seconds=3),
        Time(seconds=4),
    ]
    ros_mock.get_clock.return_value = clock_mock

    unavailable_service = Service(
        options={
            "service_name": "this_service_does_not_exist",
            "service_type": SetBool,
            "request_type": SetBool.Request,
            "response_type": SetBool.Response,
            "wait_for_response_seconds": 5.0,
            "wait_for_service_seconds": 5.0,
            "fail_if_not_available": True,
        },
        ros_node=None,
    )
    bounds = unavailable_service.calculate_utility()
    assert bounds == UtilityBounds()

    unavailable_service_2 = Service(
        options={
            "service_name": "this_service_does_not_exist",
            "service_type": SetBool,
            "request_type": SetBool.Request,
            "response_type": SetBool.Response,
            "wait_for_response_seconds": 5.0,
            "wait_for_service_seconds": 5.0,
            "fail_if_not_available": True,
        },
        ros_node=ros_mock,
    )
    bounds_2 = unavailable_service_2.calculate_utility()
    assert bounds_2 == UtilityBounds()


@mock.patch("rclpy.node.Node")
@mock.patch("rclpy.client.Client")
@mock.patch("rclpy.client.Future")
@mock.patch("rclpy.clock.Clock")
def test_node_utility(ros_mock, client_mock, future_mock, clock_mock):
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

    unavailable_service = Service(
        options={
            "service_name": "this_service_does_not_exist",
            "service_type": SetBool,
            "request_type": SetBool.Request,
            "response_type": SetBool.Response,
            "wait_for_response_seconds": 5.0,
            "wait_for_service_seconds": 5.0,
            "fail_if_not_available": True,
        },
        ros_node=ros_mock,
    )
    assert unavailable_service is not None
    unavailable_service.setup()
    assert unavailable_service.state == NodeMsg.IDLE
    unavailable_service.inputs["request"] = SetBool.Request()
    unavailable_service.tick()
    assert unavailable_service.state == NodeMsg.RUNNING

    client_mock.service_is_ready.return_value = False
    bounds = unavailable_service.calculate_utility()
    assert bounds == UtilityBounds(can_execute=False)

    client_mock.service_is_ready.return_value = True
    bounds = unavailable_service.calculate_utility()
    assert bounds == UtilityBounds(
        can_execute=True,
        has_lower_bound_success=True,
        has_upper_bound_success=True,
        has_lower_bound_failure=True,
        has_upper_bound_failure=True,
    )


@mock.patch("rclpy.node.Node")
@mock.patch("rclpy.client.Client")
@mock.patch("rclpy.client.Future")
@mock.patch("rclpy.clock.Clock")
def test_node_simulate_tick(ros_mock, client_mock, future_mock, clock_mock):
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

    unavailable_service = Service(
        options={
            "service_name": "this_service_does_not_exist",
            "service_type": SetBool,
            "request_type": SetBool.Request,
            "response_type": SetBool.Response,
            "wait_for_response_seconds": 5.0,
            "wait_for_service_seconds": 5.0,
            "fail_if_not_available": True,
        },
        ros_node=ros_mock,
    )
    assert unavailable_service is not None
    unavailable_service.setup()
    assert unavailable_service.state == NodeMsg.IDLE

    unavailable_service.inputs["request"] = SetBool.Request()

    unavailable_service.simulate_tick = True
    unavailable_service.tick()
    assert not client_mock.call_async.called
    assert unavailable_service.state == NodeMsg.RUNNING

    unavailable_service.succeed_always = True
    unavailable_service.tick()
    assert not client_mock.call_async.called
    assert unavailable_service.state == NodeMsg.SUCCEEDED
