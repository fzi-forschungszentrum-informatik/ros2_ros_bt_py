# Copyright 2026 FZI Forschungszentrum Informatik
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

from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)

from ros_bt_py.vendor.result import Result, Ok, Err

from ros_bt_py.helpers import BTNodeState

from ros_bt_py_interfaces.msg import NodeStructure, NodeState
from example_interfaces.msg import Empty

from tests.integration.conftest import TreeControlNode


class MultiPublishNode(Node):

    def __init__(self, *args, **kwargs) -> None:
        super().__init__("time_control_node", *args, **kwargs)

        self.publisher_1 = self.create_publisher(
            Empty,
            "/foo_1",
            QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                depth=1,
            ),
        )
        self.publisher_2 = self.create_publisher(
            Empty,
            "/foo_2",
            QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                depth=1,
            ),
        )
        self.publisher_3 = self.create_publisher(
            Empty,
            "/foo_3",
            QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                depth=1,
            ),
        )

    def publish_topic_1(self):
        self.publisher_1.publish(Empty())

    def publish_topic_2(self):
        self.publisher_2.publish(Empty())

    def publish_topic_3(self):
        self.publisher_3.publish(Empty())


@pytest.fixture
def multi_publish_node():
    # No rclpy init, we assume a tree_control_node to always be used
    node = MultiPublishNode()
    yield node
    node.destroy_node()


def get_node_id_map(tree_control_node: TreeControlNode) -> Result[dict[str, str], str]:
    match tree_control_node.get_tree_structure():
        case Err(e):
            return Err(e)
        case Ok(s):
            structure = s
    node_dict = {}
    node: NodeStructure
    for node in structure.nodes:
        node_dict[node.name] = node.node_id
    return Ok(node_dict)


def get_node_state(
    tree_control_node: TreeControlNode, node_id: str
) -> Result[str, str]:
    match tree_control_node.get_tree_state():
        case Err(e):
            return Err(e)
        case Ok(s):
            tree_state = s
    node_state: NodeState
    for node_state in tree_state.node_states:
        if node_state.node_id == node_id:
            return Ok(node_state.state)
    return Err(f"No node state for given id {node_id}")


def verify_node_states(
    tree_control_node: TreeControlNode, name_state_map: dict[str, BTNodeState]
):
    match get_node_id_map(tree_control_node):
        case Err(e):
            assert False, e
        case Ok(m):
            node_id_map = m
    for name, exp_state in name_state_map.items():
        assert name in node_id_map.keys()
        node_id = node_id_map[name]
        match get_node_state(tree_control_node, node_id):
            case Err(e):
                assert False, e
            case Ok(s):
                act_state = s
        assert act_state == exp_state, f"State mismatch for node {name}"
