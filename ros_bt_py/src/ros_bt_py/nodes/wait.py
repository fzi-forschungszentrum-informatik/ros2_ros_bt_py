# Copyright 2018-2023 FZI Forschungszentrum Informatik

from time import time

from ros_bt_py_interfaces.msg import Node as NodeMsg

from ros_bt_py.node import Leaf, define_bt_node
from ros_bt_py.node_config import NodeConfig


@define_bt_node(
    NodeConfig(
        version="0.1.0",
        options={"seconds_to_wait": float},
        inputs={},
        outputs={},
        max_children=0,
    )
)
class Wait(Leaf):
    """
    Returns "RUNNING" until at least the specified amount of seconds are elapsed.

    This is naturally not extremely precise because it depends on the tick interval

    If `seconds_to_wait` is 0 or negative, the node will immediately succeed
    """

    def _do_setup(self):
        self.first_tick = True
        return NodeMsg.IDLE

    def _do_tick(self):
        now = time()
        if self.first_tick:
            self.start_time = now
            self.end_time = self.start_time + self.options["seconds_to_wait"]
            self.first_tick = False
        if now >= self.end_time:
            return NodeMsg.SUCCESS
        else:
            return NodeMsg.RUNNING

    def _do_shutdown(self):
        pass

    def _do_reset(self):
        self.first_tick = True
        return NodeMsg.IDLE

    def _do_untick(self):
        return NodeMsg.IDLE


@define_bt_node(
    NodeConfig(
        version="0.1.0",
        options={},
        inputs={"seconds_to_wait": float},
        outputs={},
        max_children=0,
    )
)
class WaitInput(Leaf):
    """
    Returns "RUNNING" until at least the specified amount of seconds are elapsed.

    This is naturally not extremely precise because it depends on the tick interval

    If `seconds_to_wait` is 0 or negative, the node will immediately succeed
    """

    def _do_setup(self):
        self.first_tick = True
        return NodeMsg.IDLE

    def _do_tick(self):
        now = time()
        if self.first_tick:
            self.start_time = now
            self.end_time = self.start_time + self.inputs["seconds_to_wait"]
            self.first_tick = False
        if now >= self.end_time:
            return NodeMsg.SUCCESS
        else:
            return NodeMsg.RUNNING

    def _do_shutdown(self):
        pass

    def _do_reset(self):
        self.first_tick = True
        return NodeMsg.IDLE

    def _do_untick(self):
        return NodeMsg.IDLE
