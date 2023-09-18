# Copyright 2023 FZI Forschungszentrum Informatik

import inspect
from typing import Optional, Dict


from ros_bt_py_interfaces.msg import Node as NodeMsg
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.node import Node

from ros_bt_py.debug_manager import DebugManager
from ros_bt_py.node import Leaf, define_bt_node
from ros_bt_py.node_config import NodeConfig, OptionRef


@define_bt_node(
    NodeConfig(
        version="0.1.0",
        options={"input_type": type},
        inputs={"in": OptionRef("input_type")},
        outputs={},
        max_children=0,
    )
)
class MessageToFields(Leaf):
    def __init__(
        self,
        options: Optional[Dict] = None,
        debug_manager: Optional[DebugManager] = None,
        name: Optional[str] = None,
        ros_node: Optional[Node] = None,
        succeed_always: bool = False,
        simulate_tick: bool = False,
    ):
        super(MessageToFields, self).__init__(
            options=options,
            debug_manager=debug_manager,
            name=name,
            ros_node=ros_node,
            succeed_always=succeed_always,
            simulate_tick=simulate_tick,
        )

        node_outputs = {}

        self.passthrough = True

        if inspect.isclass(self.options["input_type"]):
            try:
                msg = self.options["input_type"]()
                for field in msg._fields_and_field_types:
                    node_outputs[field] = type(getattr(msg, field))
                self.passthrough = False
            except AttributeError:
                node_outputs["out"] = self.options["input_type"]
                self.logwarn(f"Non message type passed to: {self.name}")
        else:
            node_outputs["out"] = self.options["input_type"]

        self.node_config.extend(
            NodeConfig(options={}, inputs={}, outputs=node_outputs, max_children=0)
        )

        self._register_node_data(source_map=node_outputs, target_map=self.outputs)

    def _do_setup(self):
        return NodeMsg.IDLE

    def _do_tick(self):
        if self.passthrough:
            self.outputs["out"] = self.inputs["in"]
        else:
            for field in self.outputs:
                value = getattr(self.inputs["in"], field)
                if type(value) == tuple and self.outputs.get_type(field) == list:
                    self.outputs[field] = list(value)
                else:
                    self.outputs[field] = value
        return NodeMsg.SUCCEEDED

    def _do_untick(self):
        return NodeMsg.IDLE

    def _do_shutdown(self):
        pass

    def _do_reset(self):
        return NodeMsg.IDLE


@define_bt_node(
    NodeConfig(
        version="0.1.0",
        options={"output_type": type},
        inputs={},
        outputs={"out": OptionRef("output_type")},
        max_children=0,
    )
)
class FieldsToMessage(Leaf):
    """
    Take multiple fields as input and outputs a ROS message.

    The inputs will be named after the fields in the output message

    If the output is not as ROS message, the input will be be passed through

    """

    def __init__(
        self,
        options: Optional[Dict] = None,
        debug_manager: Optional[DebugManager] = None,
        name: Optional[str] = None,
        ros_node: Optional[Node] = None,
        succeed_always: bool = False,
        simulate_tick: bool = False,
    ):
        super(FieldsToMessage, self).__init__(
            options=options,
            debug_manager=debug_manager,
            name=name,
            ros_node=ros_node,
            succeed_always=succeed_always,
            simulate_tick=simulate_tick,
        )

        node_inputs = {}

        self.passthrough = True

        if inspect.isclass(self.options["output_type"]):
            try:
                msg = self.options["output_type"]()
                for field in msg._fields_and_field_types:
                    node_inputs[field] = type(getattr(msg, field))
                self.passthrough = False
            except AttributeError:
                node_inputs["in"] = self.options["output_type"]
                self.logwarn(f"Non message type passed to: {self.name}")
        else:
            node_inputs["in"] = self.options["output_type"]

        self.node_config.extend(
            NodeConfig(options={}, inputs=node_inputs, outputs={}, max_children=0)
        )

        self._register_node_data(source_map=node_inputs, target_map=self.inputs)

    def _do_setup(self):
        return NodeMsg.IDLE

    def _do_tick(self):
        if self.passthrough:
            self.outputs["out"] = self.inputs["in"]
        else:
            msg = self.options["output_type"]()

            for field in msg._fields_and_field_types:
                setattr(msg, field, self.inputs[field])

            self.outputs["out"] = msg
        return NodeMsg.SUCCEEDED

    def _do_untick(self):
        return NodeMsg.IDLE

    def _do_shutdown(self):
        pass

    def _do_reset(self):
        return NodeMsg.IDLE
