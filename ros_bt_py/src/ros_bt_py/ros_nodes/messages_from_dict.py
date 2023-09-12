# Copyright 2018-2023 FZI Forschungszentrum Informatik

"""BT nodes to convert Dicts to ROS messages of a specific type."""
from rosidl_runtime_py.convert import message_to_ordereddict
from ros_bt_py_interfaces.msg import Node as NodeMsg
from rosidl_runtime_py.set_message import set_message_fields

from ros_bt_py.node import Leaf, define_bt_node
from ros_bt_py.node_config import NodeConfig, OptionRef


@define_bt_node(
    NodeConfig(
        version="0.1.0",
        options={"message_type": type},
        inputs={"dict": dict},
        outputs={"message": OptionRef("message_type")},
        max_children=0,
    )
)
class MessageFromDict(Leaf):
    """Fill a ROS message with the values from `dict`."""

    def _do_setup(self):
        pass

    def _do_tick(self):
        if self.inputs.is_updated("dict") or self.state == NodeMsg.IDLE:
            message = self.options["message_type"]()
            try:
                set_message_fields(
                    message,
                    self.inputs["dict"],
                )
                self.outputs["message"] = message
            except (
                AttributeError,
                TypeError,
            ) as ex:
                self.logerr(
                    f"Error populating message of type {self.options['message_type'].__name__}: "
                    f"{str(ex)}"
                )
                return NodeMsg.FAILED
        return NodeMsg.SUCCEEDED

    def _do_shutdown(self):
        pass

    def _do_reset(self):
        return NodeMsg.IDLE

    def _do_untick(self):
        return NodeMsg.IDLE


@define_bt_node(
    NodeConfig(
        version="0.9.0",
        options={"message_type": type, "dict": dict},
        inputs={},
        outputs={"message": OptionRef("message_type")},
        max_children=0,
    )
)
class MessageFromConstDict(Leaf):
    """Fill a ROS message with the values from `dict`."""

    def _do_setup(self):
        pass

    def _do_tick(self):
        message = self.options["message_type"]()
        try:
            set_message_fields(
                message,
                self.options["dict"],
            )
            self.outputs["message"] = message
        except (TypeError, AttributeError) as ex:
            self.logerr(
                f"Error populating message of type {self.options['message_type'].__name__}: "
                f"{str(ex)}"
            )
            return NodeMsg.FAILED
        return NodeMsg.SUCCEEDED

    def _do_shutdown(self):
        pass

    def _do_reset(self):
        return NodeMsg.IDLE

    def _do_untick(self):
        return NodeMsg.IDLE
