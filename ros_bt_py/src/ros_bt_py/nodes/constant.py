# Copyright 2018-2023 FZI Forschungszentrum Informatik

from ros_bt_py_interfaces.msg import Node as NodeMsg

from ros_bt_py.node import Leaf, define_bt_node
from ros_bt_py.node_config import NodeConfig, OptionRef


@define_bt_node(
    NodeConfig(
        version="0.1.0",
        options={"constant_type": type, "constant_value": OptionRef("constant_type")},
        inputs={},
        outputs={"constant": OptionRef("constant_type")},
        max_children=0,
        tags=["constant", "value", "variable"],
    )
)
class Constant(Leaf):
    """
    Provide a set value as an output.

    Useful to provide parameters to Subtrees.
    """

    def _do_setup(self):
        pass

    def _do_tick(self):
        self.outputs["constant"] = self.options["constant_value"]
        return NodeMsg.SUCCEEDED

    def _do_shutdown(self):
        pass

    def _do_reset(self):
        return NodeMsg.IDLE

    def _do_untick(self):
        return NodeMsg.IDLE
