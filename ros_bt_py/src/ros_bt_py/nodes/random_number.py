# Copyright 2018-2023 FZI Forschungszentrum Informatik

import random

from ros_bt_py_interfaces.msg import Node as NodeMsg

from ros_bt_py.exceptions import BehaviorTreeException

from ros_bt_py.node import Leaf, define_bt_node
from ros_bt_py.node_config import NodeConfig, OptionRef


@define_bt_node(
    NodeConfig(
        version="0.1.0",
        options={"min": int, "max": int},
        inputs={},
        outputs={"random_number": int},
        max_children=0,
    )
)
class RandomInt(Leaf):
    """Provides a pseudo-random integer in range min <= random_number < max."""

    def _do_setup(self):
        validate_range(self.options["min"], self.options["max"])

    def _do_tick(self):
        validate_range(self.options["min"], self.options["max"])
        self.outputs["random_number"] = random.randrange(
            self.options["min"], self.options["max"]
        )
        return NodeMsg.SUCCEEDED

    def _do_shutdown(self):
        pass

    def _do_reset(self):
        return NodeMsg.IDLE

    def _do_untick(self):
        return NodeMsg.IDLE


@define_bt_node(
    NodeConfig(
        version="0.1.0",
        options={},
        inputs={"min": int, "max": int},
        outputs={"random_number": int},
        max_children=0,
    )
)
class RandomIntInputs(Leaf):
    """Provides a pseudo-random integer in range min <= random_number < max."""

    def _do_setup(self):
        pass

    def _do_tick(self):
        validate_range(self.inputs["min"], self.inputs["max"])
        self.outputs["random_number"] = random.randrange(
            self.inputs["min"], self.inputs["max"]
        )
        return NodeMsg.SUCCEEDED

    def _do_shutdown(self):
        pass

    def _do_reset(self):
        return NodeMsg.IDLE

    def _do_untick(self):
        return NodeMsg.IDLE


def validate_range(minimum, maximum):
    """Check if `minimum` < `maximum` and raises a BehaviorTreeException if not."""
    if minimum == maximum:
        raise BehaviorTreeException(
            f"minimum ({minimum}) cannot be equal to maximum ({maximum})"
        )
    if minimum > maximum:
        raise BehaviorTreeException(
            f"minimum ({minimum}) cannot be greater that maximum ({maximum})"
        )
