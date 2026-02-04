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

from rclpy.time import Time
from ros_bt_py.exceptions import BehaviorTreeException
from ros_bt_py.ros_nodes.param import RosParamInput
from ros_bt_py_interfaces.msg import NodeState, UtilityBounds


class TestRosParamInput:

    @pytest.fixture
    def parameter_mock(self):
        parameter_mock = mock.NonCallableMagicMock(spec_set=["value"])
        return parameter_mock

    @pytest.fixture
    def ros_mock(self, parameter_mock):
        def has_parameter_mock(name: str) -> bool:
            if name == "this_service_does_not_exist":
                return True
            else:
                return False

        clock_mock = mock.NonCallableMagicMock(spec_set=["now"])
        clock_mock.now.side_effect = [Time(seconds=0), Time(seconds=1), Time(seconds=2)]

        ros_mock = mock.NonCallableMagicMock(
            spec_set=["get_clock", "has_parameter", "get_parameter"]
        )
        ros_mock.get_clock.return_value = clock_mock
        ros_mock.has_parameter.side_effect = has_parameter_mock
        ros_mock.get_parameter.return_value = parameter_mock
        return ros_mock

    @pytest.fixture
    def target_node(self, logging_mock, ros_mock):
        node = RosParamInput(
            options={
                "param_type": str,
                "default_value": "tests",
            },
            ros_node=ros_mock,
            logging_manager=logging_mock,
        )
        return node

    def test_node_success_none(self, target_node, parameter_mock):
        parameter_mock.value = None
        target_node.setup()
        assert target_node.state == NodeState.IDLE

        target_node.inputs["param_name"] = "this_service_does_not_exist"
        target_node.tick()
        assert target_node.state == NodeState.SUCCEEDED
        assert target_node.outputs["param"] == "tests"

    def test_node_success(self, target_node, parameter_mock):
        parameter_mock.value = "not_tests"
        target_node.setup()
        assert target_node.state == NodeState.IDLE

        target_node.inputs["param_name"] = "this_service_does_not_exist"
        target_node.tick()
        assert target_node.state == NodeState.SUCCEEDED
        assert target_node.outputs["param"] == "not_tests"

        target_node.shutdown()
        assert target_node.state == NodeState.SHUTDOWN

    def test_node_failure(self, target_node):
        target_node.setup()
        assert target_node.state == NodeState.IDLE
        target_node.inputs["param_name"] = "bla"
        target_node.tick()
        assert target_node.state == NodeState.FAILURE

        target_node.shutdown()
        assert target_node.state == NodeState.SHUTDOWN

    def test_node_failure_invalid_type(self, target_node, parameter_mock):
        parameter_mock.value = 1
        target_node.setup()
        assert target_node.state == NodeState.IDLE

        target_node.inputs["param_name"] = "this_service_does_not_exist"
        target_node.tick()
        assert target_node.state == NodeState.FAILURE

    def test_node_reset(self, target_node, parameter_mock):
        parameter_mock.value = "not_tests"
        target_node.setup()
        assert target_node.state == NodeState.IDLE

        target_node.inputs["param_name"] = "this_service_does_not_exist"
        target_node.tick()
        assert target_node.state == NodeState.SUCCEEDED
        assert target_node.outputs["param"] == "not_tests"

        target_node.reset()
        assert target_node.state == NodeState.IDLE

        target_node.tick()
        assert target_node.state == NodeState.SUCCEEDED
        assert target_node.outputs["param"] == "not_tests"

        target_node.shutdown()
        assert target_node.state == NodeState.SHUTDOWN

    def test_node_untick(self, target_node, parameter_mock):
        parameter_mock.value = "not_tests"
        target_node.setup()
        assert target_node.state == NodeState.IDLE

        target_node.inputs["param_name"] = "this_service_does_not_exist"
        target_node.tick()
        assert target_node.state == NodeState.SUCCEEDED
        assert target_node.outputs["param"] == "not_tests"

        target_node.untick()
        assert target_node.state == NodeState.IDLE

        target_node.tick()
        assert target_node.state == NodeState.SUCCEEDED
        assert target_node.outputs["param"] == "not_tests"

        target_node.shutdown()
        assert target_node.state == NodeState.SHUTDOWN

    def test_node_no_ros(self, logging_mock, error_log):
        param_node = RosParamInput(
            options={
                "param_type": str,
                "default_value": "tests",
            },
            ros_node=None,
            logging_manager=logging_mock,
        )
        with pytest.warns(error_log, match=".*no.*ROS node.*"):
            setup_result = param_node.setup()
        assert isinstance(setup_result.err(), BehaviorTreeException)

    def test_node_utility(self, logging_mock):

        param_node = RosParamInput(
            options={
                "param_type": str,
                "default_value": "tests",
            },
            ros_node=None,
            logging_manager=logging_mock,
        )
        utility_result = param_node.calculate_utility()
        assert utility_result.ok() == UtilityBounds()
