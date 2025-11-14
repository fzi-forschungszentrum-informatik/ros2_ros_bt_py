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
import uuid
from ros_bt_py.ros_helpers import ros_to_uuid, uuid_to_ros
from ros_bt_py.subtree_manager import SubtreeManager
from ros_bt_py_interfaces.msg import (
    TreeStructure,
    TreeState,
    TreeData,
    NodeStructure,
    NodeState,
    WiringData,
    Wiring,
    NodeDataLocation,
)


class TestSubtreeManager:
    @pytest.fixture
    def subtree_manager_no_publish(self):
        subtree_manager = SubtreeManager()
        return subtree_manager

    @pytest.fixture
    def subtree_manager(self):
        subtree_manager = SubtreeManager()
        subtree_manager.publish_subtrees = True
        return subtree_manager

    def test_add_subtree_structure(self, subtree_manager: SubtreeManager):
        subtree_fields = [
            {
                "tree_id": uuid_to_ros(uuid.UUID(int=10)),
                "tick_frequency_hz": 42.5,
                "nodes": [
                    NodeStructure(
                        node_id=uuid_to_ros(uuid.UUID(int=100)),
                        child_ids=[
                            uuid_to_ros(uuid.UUID(int=1000)),
                            uuid_to_ros(uuid.UUID(int=1001)),
                        ],
                    )
                ],
            },
            {
                "tree_id": uuid_to_ros(uuid.UUID(int=11)),
                "tick_frequency_hz": 42.0,
                "nodes": [
                    NodeStructure(
                        node_id=uuid_to_ros(uuid.UUID(int=110)),
                        child_ids=[
                            uuid_to_ros(uuid.UUID(int=1100)),
                            uuid_to_ros(uuid.UUID(int=1101)),
                        ],
                    )
                ],
            },
        ]
        subtrees = [TreeStructure(**fields) for fields in subtree_fields]

        subtree_manager_subtree_structures = subtree_manager.get_subtree_structures()
        assert subtree_manager_subtree_structures == []

        for tree in subtrees:
            subtree_manager.add_subtree_structure(
                ros_to_uuid(tree.tree_id).unwrap(), tree
            )

        subtree_manager_subtree_structures = subtree_manager.get_subtree_structures()
        assert len(subtree_manager_subtree_structures) == len(subtrees)
        for subtree_manager_tree, tree, fields in zip(
            subtree_manager_subtree_structures, subtrees, subtree_fields
        ):
            for field in fields.keys():
                assert getattr(subtree_manager_tree, field) == getattr(tree, field)

    def test_add_subtree_state(self, subtree_manager: SubtreeManager):
        subtree_fields = [
            {
                "tree_id": uuid_to_ros(uuid.UUID(int=10)),
                "state": TreeState.EDITABLE,
                "node_states": [
                    NodeState(
                        node_id=uuid_to_ros(uuid.UUID(int=100)),
                        state=NodeState.SHUTDOWN,
                    )
                ],
            },
            {
                "tree_id": uuid_to_ros(uuid.UUID(int=11)),
                "state": TreeState.EDITABLE,
                "node_states": [
                    NodeState(
                        node_id=uuid_to_ros(uuid.UUID(int=110)),
                        state=NodeState.SHUTDOWN,
                    )
                ],
            },
        ]
        subtrees = [TreeState(**fields) for fields in subtree_fields]

        subtree_manager_subtree_states = subtree_manager.get_subtree_states()
        assert subtree_manager_subtree_states == []

        for tree in subtrees:
            subtree_manager.add_subtree_state(ros_to_uuid(tree.tree_id).unwrap(), tree)

        subtree_manager_subtree_states = subtree_manager.get_subtree_states()
        assert len(subtree_manager_subtree_states) == len(subtrees)
        for subtree_manager_tree, tree, fields in zip(
            subtree_manager_subtree_states, subtrees, subtree_fields
        ):
            for field in fields.keys():
                assert getattr(subtree_manager_tree, field) == getattr(tree, field)

    def test_add_subtree_data(self, subtree_manager: SubtreeManager):
        subtree_fields = [
            {
                "tree_id": uuid_to_ros(uuid.UUID(int=10)),
                "wiring_data": [
                    WiringData(
                        wiring=Wiring(
                            source=NodeDataLocation(
                                node_id=uuid_to_ros(uuid.UUID(int=100)),
                                data_kind=NodeDataLocation.OUTPUT_DATA,
                                data_key="output1",
                            ),
                            target=NodeDataLocation(
                                node_id=uuid_to_ros(uuid.UUID(int=101)),
                                data_kind=NodeDataLocation.INPUT_DATA,
                                data_key="input1",
                            ),
                        ),
                        serialized_data="1",
                        serialized_type="int",
                        serialized_expected_type="int",
                    )
                ],
            },
            {
                "tree_id": uuid_to_ros(uuid.UUID(int=11)),
                "wiring_data": [
                    WiringData(
                        wiring=Wiring(
                            source=NodeDataLocation(
                                node_id=uuid_to_ros(uuid.UUID(int=110)),
                                data_kind=NodeDataLocation.OUTPUT_DATA,
                                data_key="output1",
                            ),
                            target=NodeDataLocation(
                                node_id=uuid_to_ros(uuid.UUID(int=111)),
                                data_kind=NodeDataLocation.INPUT_DATA,
                                data_key="input1",
                            ),
                        ),
                        serialized_data="1",
                        serialized_type="int",
                        serialized_expected_type="int",
                    )
                ],
            },
        ]
        subtrees = [TreeData(**fields) for fields in subtree_fields]

        subtree_manager_subtree_data = subtree_manager.get_subtree_data()
        assert subtree_manager_subtree_data == []

        for tree in subtrees:
            subtree_manager.add_subtree_data(ros_to_uuid(tree.tree_id).unwrap(), tree)

        subtree_manager_subtree_data = subtree_manager.get_subtree_data()
        assert len(subtree_manager_subtree_data) == len(subtrees)
        for subtree_manager_tree, tree, fields in zip(
            subtree_manager_subtree_data, subtrees, subtree_fields
        ):
            for field in fields.keys():
                assert getattr(subtree_manager_tree, field) == getattr(tree, field)

    def test_clear_subtrees(self, subtree_manager: SubtreeManager):
        subtree_manager.add_subtree_structure(uuid.UUID(int=10), TreeStructure())
        subtree_manager.add_subtree_state(uuid.UUID(int=10), TreeState())
        subtree_manager.add_subtree_data(uuid.UUID(int=10), TreeData())
        subtree_manager.add_subtree_structure(uuid.UUID(int=11), TreeStructure())
        subtree_manager.add_subtree_state(uuid.UUID(int=11), TreeState())
        subtree_manager.add_subtree_data(uuid.UUID(int=11), TreeData())
        subtree_manager_subtree_structures = subtree_manager.get_subtree_structures()
        subtree_manager_subtree_states = subtree_manager.get_subtree_states()
        subtree_manager_subtree_data = subtree_manager.get_subtree_data()
        assert len(subtree_manager_subtree_structures) > 0
        assert len(subtree_manager_subtree_states) > 0
        assert len(subtree_manager_subtree_data) > 0
        subtree_manager.clear_subtrees()
        subtree_manager_subtree_structures = subtree_manager.get_subtree_structures()
        subtree_manager_subtree_states = subtree_manager.get_subtree_states()
        subtree_manager_subtree_data = subtree_manager.get_subtree_data()
        assert subtree_manager_subtree_structures == []
        assert subtree_manager_subtree_states == []
        assert subtree_manager_subtree_data == []

    def test_remove_subtree(self, subtree_manager: SubtreeManager):
        subtree_manager.add_subtree_structure(uuid.UUID(int=10), TreeStructure())
        subtree_manager.add_subtree_state(uuid.UUID(int=10), TreeState())
        subtree_manager.add_subtree_data(uuid.UUID(int=10), TreeData())
        subtree_manager.add_subtree_structure(uuid.UUID(int=11), TreeStructure())
        subtree_manager.add_subtree_state(uuid.UUID(int=11), TreeState())
        subtree_manager.add_subtree_data(uuid.UUID(int=11), TreeData())
        subtree_manager_subtree_structures = subtree_manager.get_subtree_structures()
        subtree_manager_subtree_states = subtree_manager.get_subtree_states()
        subtree_manager_subtree_data = subtree_manager.get_subtree_data()
        assert len(subtree_manager_subtree_structures) == 2
        assert len(subtree_manager_subtree_states) == 2
        assert len(subtree_manager_subtree_data) == 2
        subtree_manager.remove_subtree(uuid.UUID(int=10))
        subtree_manager_subtree_structures = subtree_manager.get_subtree_structures()
        subtree_manager_subtree_states = subtree_manager.get_subtree_states()
        subtree_manager_subtree_data = subtree_manager.get_subtree_data()
        assert len(subtree_manager_subtree_structures) == 1
        assert len(subtree_manager_subtree_states) == 1
        assert len(subtree_manager_subtree_data) == 1

    def test_check_publish_subtrees_false(
        self, subtree_manager_no_publish: SubtreeManager
    ):
        assert not subtree_manager_no_publish.publish_subtrees

    def test_check_publish_subtrees_true(self, subtree_manager: SubtreeManager):
        assert subtree_manager.publish_subtrees
