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

from tests.conftest import ErrorLog

from ros_bt_py.custom_types import RosServiceType
from ros_bt_py.ros_nodes.service import WaitForServiceInput
from rclpy.time import Time
from ros_bt_py_interfaces.msg import NodeState
from ros_bt_py.exceptions import BehaviorTreeException


class TestWaitForService:

    @pytest.fixture
    def clock_mock(self):
        clock_mock = mock.NonCallableMagicMock(spec_set=["now"])
        clock_mock.now.side_effect = [Time(seconds=sec) for sec in range(20)]
        return clock_mock

    @pytest.fixture
    def client_mock(self):
        client_mock = mock.NonCallableMagicMock(spec_set=["service_is_ready"])
        client_mock.service_is_ready.return_value = False
        return client_mock

    @pytest.fixture
    def ros_mock(self, client_mock, clock_mock):
        ros_mock = mock.NonCallableMagicMock(
            spec_set=["create_client", "destroy_client", "get_clock"]
        )
        ros_mock.create_client.return_value = client_mock
        ros_mock.get_clock.return_value = clock_mock
        return ros_mock

    @pytest.fixture
    def target_node(self, logging_mock, ros_mock):
        node = WaitForServiceInput(
            options={
                "service_type": RosServiceType("std_srvs/srv/SetBool"),
                "wait_for_service_seconds": 5.0,
            },
            ros_node=ros_mock,
            logging_manager=logging_mock,
        )
        return node

    def test_node_success(self, target_node, ros_mock, client_mock, clock_mock):
        assert target_node.setup().is_ok()
        assert target_node.state == NodeState.IDLE

        target_node.inputs["service_name"] = "this_service_does_not_exist"
        assert target_node.tick().is_ok()
        assert target_node.state == NodeState.RUNNING

        client_mock.service_is_ready.return_value = True
        assert target_node.tick().is_ok()
        assert target_node.state == NodeState.SUCCEEDED

        assert target_node.shutdown().is_ok()
        assert target_node.state == NodeState.SHUTDOWN
        assert ros_mock.destroy_client.called

    def test_node_failure(self, target_node, ros_mock, client_mock, clock_mock):
        # Take control of clock_mock to simulate timeout
        clock_mock.now.side_effect = None
        clock_mock.now.return_value = Time(seconds=0)

        assert target_node.setup().is_ok()
        assert target_node.state == NodeState.IDLE

        target_node.inputs["service_name"] = "this_service_does_not_exist"
        assert target_node.tick().is_ok()
        assert target_node.state == NodeState.RUNNING

        clock_mock.now.return_value = Time(seconds=10)
        assert target_node.tick().is_ok()
        assert target_node.state == NodeState.FAILED

        assert target_node.shutdown().is_ok()
        assert target_node.state == NodeState.SHUTDOWN
        assert ros_mock.destroy_client.called

    def test_node_reset(self, target_node, ros_mock, client_mock, clock_mock):
        # Set custom clock progression
        clock_mock.now.side_effect = [
            Time(seconds=0),
            Time(seconds=1),
            Time(seconds=10),
            Time(seconds=11),
            Time(seconds=12),
        ]

        assert target_node.setup().is_ok()
        assert target_node.state == NodeState.IDLE

        target_node.inputs["service_name"] = "this_service_does_not_exist"
        assert target_node.tick().is_ok()
        assert target_node.state == NodeState.RUNNING

        assert target_node.reset().is_ok()
        assert target_node.state == NodeState.IDLE
        assert target_node._last_service_call_time is None  # type: ignore

        assert target_node.tick().is_ok()
        assert target_node.state == NodeState.RUNNING

        assert target_node.tick().is_ok()
        assert target_node.state == NodeState.RUNNING

        assert target_node.shutdown().is_ok()
        assert target_node.state == NodeState.SHUTDOWN
        assert ros_mock.destroy_client.called

    def test_node_untick(self, target_node, ros_mock, client_mock, clock_mock):
        # Set custom clock progression
        clock_mock.now.side_effect = [
            Time(seconds=0),
            Time(seconds=1),
            Time(seconds=10),
            Time(seconds=11),
            Time(seconds=12),
        ]

        assert target_node.setup().is_ok()
        assert target_node.state == NodeState.IDLE

        target_node.inputs["service_name"] = "this_service_does_not_exist"
        assert target_node.tick().is_ok()
        assert target_node.state == NodeState.RUNNING

        assert target_node.untick().is_ok()
        assert target_node.state == NodeState.IDLE
        assert target_node._last_service_call_time is None  # type: ignore

        assert target_node.tick().is_ok()
        assert target_node.state == NodeState.RUNNING

        assert target_node.tick().is_ok()
        assert target_node.state == NodeState.RUNNING

        assert target_node.shutdown().is_ok()
        assert target_node.state == NodeState.SHUTDOWN
        assert ros_mock.destroy_client.called

    def test_node_no_ros(self, logging_mock):
        unavailable_service = WaitForServiceInput(
            options={
                "service_type": RosServiceType("std_srvs/srv/SetBool"),
                "wait_for_service_seconds": 5.0,
            },
            ros_node=None,
            logging_manager=logging_mock,
        )
        with pytest.warns(ErrorLog, match=".*No ROS node.*"):
            setup_result = unavailable_service.setup()
        assert setup_result.is_err()
        assert isinstance(setup_result.err(), BehaviorTreeException)
