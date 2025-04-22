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
from ros_bt_py.subtree_manager import SubtreeManager
from ros_bt_py_interfaces.msg import Node, Tree
from ros_bt_py.exceptions import BehaviorTreeException
import uuid
from ros_bt_py.ros_helpers import uuid_to_ros


class TestSubtreeManager:
    @pytest.fixture
    def subtree_manager_no_publish(self):
        subtree_manager = SubtreeManager()
        return subtree_manager

    @pytest.fixture
    def subtree_manager(self):
        subtree_manager = SubtreeManager()
        subtree_manager.set_publish_subtrees(
            publish_subtrees=True,
        )
        return subtree_manager

    def test_add_subtree_info(self, subtree_manager):
        node_ids = [uuid.uuid4(), uuid.uuid4()]
        subtree_fields = [
            {
                "name": "subtree_0",
                "tick_frequency_hz": 42.5,
                "state": Tree.IDLE,
                "nodes": [
                    Node(
                        name="node_0.0",
                        node_id=uuid_to_ros(node_ids[0]),
                        state=Node.IDLE,
                        child_node_ids=[
                            uuid_to_ros(uuid.uuid4()),
                            uuid_to_ros(uuid.uuid4()),
                        ],
                    )
                ],
            },
            {
                "name": "subtree_1",
                "tick_frequency_hz": 42.0,
                "state": Tree.TICKING,
                "nodes": [
                    Node(
                        name="node_1.0",
                        node_id=uuid_to_ros(node_ids[1]),
                        state=Node.RUNNING,
                        child_node_ids=[
                            uuid_to_ros(uuid.uuid4()),
                            uuid_to_ros(uuid.uuid4()),
                        ],
                    )
                ],
            },
        ]
        subtrees = [Tree(**fields) for fields in subtree_fields]

        subtree_manager_subtree_states = (
            subtree_manager.get_subtree_info_msg().subtree_states
        )
        assert subtree_manager_subtree_states == []

        for i, tree in enumerate(subtrees):
            subtree_manager.add_subtree_info(node_ids[i], tree)

        subtree_manager_subtree_states = (
            subtree_manager.get_subtree_info_msg().subtree_states
        )
        assert len(subtree_manager_subtree_states) == len(subtrees)
        for subtree_manager_tree, tree, fields in zip(
            subtree_manager_subtree_states, subtrees, subtree_fields
        ):
            for field in fields.keys():
                assert getattr(subtree_manager_tree, field) == getattr(tree, field)

    def test_clear_subtrees(self, subtree_manager):
        test_id = uuid.uuid4()
        subtree_manager.add_subtree_info(test_id, Tree())
        subtree_manager_subtree_states = (
            subtree_manager.get_subtree_info_msg().subtree_states
        )
        assert len(subtree_manager_subtree_states) > 0
        subtree_manager.clear_subtrees()
        subtree_manager_subtree_states = (
            subtree_manager.get_subtree_info_msg().subtree_states
        )
        assert subtree_manager_subtree_states == []

    def test_check_publish_subtrees_false(
        self, subtree_manager_no_publish: SubtreeManager
    ):
        assert not subtree_manager_no_publish.get_publish_subtrees()

    def test_check_publish_subtrees_true(self, subtree_manager: SubtreeManager):
        assert subtree_manager.get_publish_subtrees()

    def test_add_subtree_info_exception(
        self, subtree_manager_no_publish: SubtreeManager
    ):
        test_id = uuid.uuid4()
        with pytest.raises(BehaviorTreeException):
            subtree_manager_no_publish.add_subtree_info(test_id, Tree())
