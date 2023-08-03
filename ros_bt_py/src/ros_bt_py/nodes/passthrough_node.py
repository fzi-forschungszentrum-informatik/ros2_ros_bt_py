# Copyright 2018-2023 FZI Forschungszentrum Informatik

from ros_bt_py_interfaces.msg import Node as NodeMsg

from ros_bt_py.node import Leaf, define_bt_node
from ros_bt_py.node_config import NodeConfig, OptionRef


@define_bt_node(
    NodeConfig(
        version="0.1.0",
        options={"passthrough_type": type},
        inputs={"in": OptionRef("passthrough_type")},
        outputs={"out": OptionRef("passthrough_type")},
        max_children=0,
    )
)
class PassthroughNode(Leaf):
    """
    Pass through a piece of data.

    Useful for testing, and to mark the inputs of a BT that is meant
    to be loaded as a subtree.

    """

    def _do_setup(self):
        return NodeMsg.IDLE

    def _do_tick(self):
        self.outputs["out"] = self.inputs["in"]
        return NodeMsg.SUCCEEDED

    def _do_untick(self):
        return NodeMsg.IDLE

    def _do_shutdown(self):
        pass

    def _do_reset(self):
        self.outputs["out"] = None
        self.outputs.reset_updated()
        return NodeMsg.IDLE
