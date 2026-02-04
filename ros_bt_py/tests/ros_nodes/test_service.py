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
from ros_bt_py.helpers import BTNodeState
from ros_bt_py.custom_types import RosServiceName, RosServiceType
from ros_bt_py.ros_nodes.service import Service
from rclpy.time import Time
from ros_bt_py_interfaces.msg import NodeState, UtilityBounds
from ros_bt_py.exceptions import BehaviorTreeException


class TestService:

    @pytest.fixture
    def clock_mock(self):
        clock_mock = mock.NonCallableMagicMock(spec_set=["now"])
        clock_mock.now.side_effect = [Time(seconds=sec) for sec in range(20)]
        return clock_mock

    @pytest.fixture
    def future_mock(self):
        future_mock = mock.NonCallableMagicMock(
            spec_set=["cancel", "cancelled", "done", "result"]
        )
        future_mock.cancelled.return_value = False
        future_mock.done.return_value = False
        response = SetBool.Response()
        response.success = True
        future_mock.result.return_value = response
        return future_mock

    @pytest.fixture
    def client_mock(self, future_mock):
        client_mock = mock.NonCallableMagicMock(
            spec_set=["call_async", "service_is_ready", "wait_for_service"]
        )
        client_mock.call_async.return_value = future_mock
        client_mock.service_is_ready.return_value = True
        client_mock.wait_for_service.return_value = True
        return client_mock

    @pytest.fixture
    def ros_mock(self, clock_mock, client_mock):
        ros_mock = mock.NonCallableMagicMock(
            spec_set=["create_client", "destroy_client", "get_clock"]
        )
        ros_mock.create_client.return_value = client_mock
        ros_mock.get_clock.return_value = clock_mock
        return ros_mock

    @pytest.fixture
    def target_node(self, logging_mock, ros_mock):
        node = Service(
            options={
                "service_name": RosServiceName(name="this_service_does_not_exist"),
                "service_type": RosServiceType("std_srvs/srv/SetBool"),
                "wait_for_response_seconds": 5.0,
                "wait_for_service_seconds": 5.0,
                "fail_if_not_available": True,
            },
            ros_node=ros_mock,
            logging_manager=logging_mock,
        )
        return node

    def test_node_success(self, target_node, ros_mock, client_mock, future_mock):
        assert target_node.setup().is_ok()
        assert target_node.state == BTNodeState.IDLE

        request = SetBool.Request()
        request.data = False
        target_node.inputs["data"] = request.data
        assert target_node.tick().is_ok()
        assert target_node.state == BTNodeState.RUNNING
        client_mock.call_async.assert_called_with(request)

        future_mock.done.return_value = True
        assert target_node.tick().is_ok()
        assert target_node.state == BTNodeState.SUCCEEDED
        assert target_node.outputs["success"]

        assert target_node.shutdown().is_ok()
        assert target_node.state == BTNodeState.SHUTDOWN
        assert ros_mock.destroy_client.called

    def test_node_failure(self, target_node, ros_mock, client_mock, future_mock):
        assert target_node.setup().is_ok()
        assert target_node.state == BTNodeState.IDLE

        request = SetBool.Request()
        request.data = False
        target_node.inputs["data"] = request.data
        assert target_node.tick().is_ok()
        assert target_node.state == BTNodeState.RUNNING
        client_mock.call_async.assert_called_with(request)

        future_mock.cancelled.return_value = True
        assert target_node.tick().is_ok()
        assert target_node.state == BTNodeState.FAILED

        assert target_node.shutdown().is_ok()
        assert target_node.state == BTNodeState.SHUTDOWN
        assert ros_mock.destroy_client.called

    def test_node_timeout(
        self, target_node, ros_mock, client_mock, clock_mock, warn_log
    ):
        # Take control of clock_mock to simulate timeout
        clock_mock.now.side_effect = None
        clock_mock.now.return_value = Time(seconds=0)

        assert target_node.setup().is_ok()
        assert target_node.state == BTNodeState.IDLE

        request = SetBool.Request()
        request.data = False
        target_node.inputs["data"] = request.data
        assert target_node.tick().is_ok()
        assert target_node.state == BTNodeState.RUNNING
        client_mock.call_async.assert_called_with(request)

        clock_mock.now.return_value = Time(seconds=10)
        with pytest.warns(warn_log, match=".*timed out.*"):
            assert target_node.tick().is_ok()
        assert target_node.state == BTNodeState.FAILED

        assert target_node.shutdown().is_ok()
        assert target_node.state == BTNodeState.SHUTDOWN
        assert ros_mock.destroy_client.called

    def test_node_reset_shutdown(self, target_node, ros_mock, future_mock):
        assert target_node.setup().is_ok()
        assert target_node.state == BTNodeState.IDLE

        target_node.inputs["data"] = SetBool.Request().data
        assert target_node.tick().is_ok()
        assert target_node.state == BTNodeState.RUNNING

        future_mock.done.return_value = True
        assert target_node.tick().is_ok()
        assert target_node.state == BTNodeState.SUCCEEDED
        assert target_node.outputs["success"]

        assert target_node.reset().is_ok()
        assert target_node.state == BTNodeState.IDLE

        assert target_node.shutdown().is_ok()
        assert target_node.state == BTNodeState.SHUTDOWN
        assert ros_mock.destroy_client.called

    def test_node_reset(self, target_node, future_mock):
        assert target_node.setup().is_ok()
        assert target_node.state == BTNodeState.IDLE

        target_node.inputs["data"] = SetBool.Request().data
        assert target_node.tick().is_ok()
        assert target_node.state == BTNodeState.RUNNING

        future_mock.done.return_value = True
        assert target_node.tick().is_ok()
        assert target_node.state == BTNodeState.SUCCEEDED
        assert target_node.outputs["success"]

        assert target_node.reset().is_ok()

        # Reset future, change response
        future_mock.done.return_value = False
        response = SetBool.Response()
        response.success = False
        future_mock.result.return_value = response

        assert target_node.state == BTNodeState.IDLE
        target_node.inputs["data"] = SetBool.Request().data
        assert target_node.tick().is_ok()
        assert target_node.state == BTNodeState.RUNNING

        future_mock.done.return_value = True
        assert target_node.tick().is_ok()
        assert target_node.state == BTNodeState.SUCCEEDED
        assert not target_node.outputs["success"]

    def test_node_untick(self, target_node, future_mock):
        assert target_node.setup().is_ok()
        assert target_node.state == BTNodeState.IDLE

        target_node.inputs["data"] = SetBool.Request().data
        assert target_node.tick().is_ok()
        assert target_node.state == BTNodeState.RUNNING

        assert target_node.untick().is_ok()
        assert target_node.state == BTNodeState.IDLE
        assert future_mock.cancel.called

    def test_node_no_ros(self, logging_mock, error_log):
        unavailable_service = Service(
            options={
                "service_name": RosServiceName(name="this_service_does_not_exist"),
                "service_type": RosServiceType("std_srvs/srv/SetBool"),
                "wait_for_response_seconds": 5.0,
                "wait_for_service_seconds": 5.0,
                "fail_if_not_available": True,
            },
            ros_node=None,
            logging_manager=logging_mock,
        )
        assert unavailable_service is not None
        with pytest.warns(error_log, match=".*No ROS node.*"):
            result = unavailable_service.setup()
        assert result.is_err()
        assert isinstance(result.unwrap_err(), BehaviorTreeException)

    def test_node_utility(self, target_node, client_mock, warn_log):
        assert target_node.setup().is_ok()
        assert target_node.state == BTNodeState.IDLE

        target_node.inputs["data"] = SetBool.Request().data
        assert target_node.tick().is_ok()
        assert target_node.state == BTNodeState.RUNNING

        client_mock.service_is_ready.return_value = False
        with pytest.warns(warn_log, match=".*unavailable.*"):
            bounds_result = target_node.calculate_utility()
        assert bounds_result.is_ok()
        assert bounds_result.unwrap() == UtilityBounds(can_execute=False)

        client_mock.service_is_ready.return_value = True
        bounds_result = target_node.calculate_utility()
        assert bounds_result.is_ok()
        assert bounds_result.unwrap() == UtilityBounds(
            can_execute=True,
            has_lower_bound_success=True,
            has_upper_bound_success=True,
            has_lower_bound_failure=True,
            has_upper_bound_failure=True,
        )

    def test_node_utility_no_ros(self, logging_mock, target_node, warn_log):
        target_node_no_ros = Service(
            options={
                "service_name": RosServiceName(name="this_service_does_not_exist"),
                "service_type": RosServiceType("std_srvs/srv/SetBool"),
                "wait_for_response_seconds": 5.0,
                "wait_for_service_seconds": 5.0,
                "fail_if_not_available": True,
            },
            ros_node=None,
            logging_manager=logging_mock,
        )
        with pytest.warns(warn_log, match=".*no.*ros node.*"):
            bounds_result = target_node_no_ros.calculate_utility()
        assert bounds_result.is_ok()
        assert bounds_result.unwrap() == UtilityBounds()

        with pytest.warns(warn_log, match=".*no.*service.*"):
            bounds_2_result = target_node.calculate_utility()
        assert bounds_2_result.is_ok()
        assert bounds_2_result.unwrap() == UtilityBounds()
