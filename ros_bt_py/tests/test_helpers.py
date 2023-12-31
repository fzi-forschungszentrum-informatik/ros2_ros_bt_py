from ros_bt_py.helpers import (
    loglevel_is,
    get_default_value,
    rospy_log_level_to_logging_log_level,
    remove_input_output_values,
)
from ros_bt_py_interfaces.msg import Tree, Node, NodeData
import logging
import rclpy
import rclpy.logging
import pytest
import unittest.mock as mock
from collections import OrderedDict
from ros_bt_py.ros_helpers import EnumValue, LoggerLevel


class TestHelpers:
    @pytest.mark.parametrize(
        "ros_log_level, expected_log_level",
        [
            (rclpy.logging.LoggingSeverity.DEBUG, logging.DEBUG),
            (rclpy.logging.LoggingSeverity.INFO, logging.INFO),
            (rclpy.logging.LoggingSeverity.WARN, logging.WARNING),
            (rclpy.logging.LoggingSeverity.ERROR, logging.ERROR),
            (rclpy.logging.LoggingSeverity.FATAL, logging.FATAL),
        ],
    )
    def test_rospy_log_level_to_logging_log_level(
        self, ros_log_level, expected_log_level
    ):
        result = rospy_log_level_to_logging_log_level(ros_log_level)
        assert result == expected_log_level

    @pytest.mark.parametrize(
        "effective_level, input_level, expected_result",
        [
            (logging.DEBUG, logging.DEBUG, True),
            (logging.INFO, logging.INFO, True),
            (logging.WARNING, logging.WARNING, True),
            (logging.ERROR, logging.ERROR, True),
            (logging.FATAL, logging.FATAL, True),
            (logging.DEBUG, logging.FATAL, True),
            (logging.INFO, logging.FATAL, True),
            (logging.WARN, logging.FATAL, True),
            (logging.ERROR, logging.FATAL, True),
            (logging.FATAL, logging.ERROR, False),
            (logging.INFO, logging.DEBUG, False),
        ],
    )
    @mock.patch("logging.getLogger")
    def test_log_level_is(
        self, mock_logger, effective_level, input_level, expected_result
    ):
        mock_logger.return_value.getEffectiveLevel.return_value = effective_level
        result = loglevel_is(input_level)
        assert result == expected_result

    def test_remove_input_output_values(self):
        nodes = [
            Node(
                inputs=[
                    NodeData(serialized_value=f"some_value_{i}") for i in range(10)
                ],
                outputs=[
                    NodeData(serialized_value=f"some_value_{i}") for i in range(10)
                ],
            )
            for _ in range(10)
        ]
        input_tree = Tree()
        input_tree.nodes = nodes
        output_tree = remove_input_output_values(input_tree)
        for node in output_tree.nodes:
            for node_input in node.inputs:
                assert node_input.serialized_value == "null"
            for node_output in node.outputs:
                assert node_output.serialized_value == "null"

    @pytest.mark.parametrize(
        "data_type, expected_output, ros",
        [
            (type, int, False),
            (int, 0, False),
            (str, "foo", False),
            (float, 1.2, False),
            (bool, False, False),
            (list, [], False),
            (dict, {}, False),
            (type, int, True),
            (list, [], True),
        ],
    )
    def test_get_default_value_basic_data_types(self, data_type, expected_output, ros):
        output = get_default_value(data_type, ros)
        assert output == expected_output

    @pytest.mark.parametrize(
        "data_type, ros",
        [
            (OrderedDict, False),
            (LoggerLevel, False),
            (EnumValue, False),
            (OrderedDict, True),
        ],
    )
    def test_get_default_value_defined_class_data_types(self, data_type, ros):
        output = get_default_value(data_type, ros)
        assert isinstance(output, data_type)

    @pytest.mark.parametrize(
        "data_type, expected_output, ros",
        [(mock.Mock(), mock.Mock, True), (mock.Mock(), {}, False)],
    )
    def test_get_default_value_other_class_data_types(
        self, data_type, expected_output, ros
    ):
        output = get_default_value(data_type, ros)
        if ros:
            assert isinstance(output, expected_output)
        else:
            assert output == expected_output
