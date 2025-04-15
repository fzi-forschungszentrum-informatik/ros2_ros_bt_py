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
from ros_bt_py_interfaces.msg import (
    TreeStructure, TreeState, TreeData,
    NodeStructure, NodeState, 
    WiringData, Wiring,
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
        subtree_manager.set_publish_subtrees(
            publish_subtrees=True,
        )
        return subtree_manager

    def test_add_subtree_structure(self, subtree_manager: SubtreeManager):
        subtree_fields = [
            {
                "name": "subtree_0",
                "tick_frequency_hz": 42.5,
                "nodes": [
                    NodeStructure(
                        name="node_0.0",
                        child_names=["node_0.0.0", "node_0.0.1"],
                    )
                ],
            },
            {
                "name": "subtree_1",
                "tick_frequency_hz": 42.0,
                "nodes": [
                    NodeStructure(
                        name="node_1.0",
                        child_names=["node_1.0.0", "node_1.0.1"],
                    )
                ],
            },
        ]
        subtrees = [TreeStructure(**fields) for fields in subtree_fields]

        subtree_manager_subtree_structures = (
            subtree_manager.get_subtree_structures()
        )
        assert subtree_manager_subtree_structures == []

        for i, tree in enumerate(subtrees):
            subtree_manager.add_subtree_structure("node_" + str(i), tree)

        subtree_manager_subtree_structures = (
            subtree_manager.get_subtree_structures()
        )
        assert len(subtree_manager_subtree_structures) == len(subtrees)
        for subtree_manager_tree, tree, fields in zip(
            subtree_manager_subtree_structures, subtrees, subtree_fields
        ):
            for field in fields.keys():
                assert getattr(subtree_manager_tree, field) == getattr(tree, field)

    def test_add_subtree_state(self, subtree_manager: SubtreeManager):
        subtree_fields = [
            {
                "state": TreeState.EDITABLE,
                "node_states": [
                    NodeState(
                        name="node_0.0",
                        state=NodeState.SHUTDOWN
                    )
                ]
            },
            {
                "state": TreeState.EDITABLE,
                "node_states": [
                    NodeState(
                        name="node_1.0",
                        state=NodeState.SHUTDOWN
                    )
                ]
            },
        ]
        subtrees = [TreeState(**fields) for fields in subtree_fields]

        subtree_manager_subtree_states = (
            subtree_manager.get_subtree_states()
        )
        assert subtree_manager_subtree_states == []

        for i, tree in enumerate(subtrees):
            subtree_manager.add_subtree_state("node_" + str(i), tree)

        subtree_manager_subtree_states = (
            subtree_manager.get_subtree_states()
        )
        assert len(subtree_manager_subtree_states) == len(subtrees)
        for subtree_manager_tree, tree, fields in zip(
            subtree_manager_subtree_states, subtrees, subtree_fields
        ):
            for field in fields.keys():
                assert getattr(subtree_manager_tree, field) == getattr(tree, field)

    def test_add_subtree_data(self, subtree_manager: SubtreeManager):
        subtree_fields = [
            {
                "wiring_data": [
                    WiringData(
                        wiring=Wiring(
                            source=NodeDataLocation(
                                node_name="node_0.0",
                                data_kind=NodeDataLocation.OUTPUT_DATA,
                                data_key="output1"
                            ),
                            target=NodeDataLocation(
                                node_name="node_0.1",
                                data_kind=NodeDataLocation.INPUT_DATA,
                                data_key="input1"
                            ),
                        ),
                        serialized_data="1",
                        serialized_type="int",
                        serialized_expected_type="int",
                    )
                ]
            },
            {
                "wiring_data": [
                    WiringData(
                        wiring=Wiring(
                            source=NodeDataLocation(
                                node_name="node_1.0",
                                data_kind=NodeDataLocation.OUTPUT_DATA,
                                data_key="output1"
                            ),
                            target=NodeDataLocation(
                                node_name="node_1.1",
                                data_kind=NodeDataLocation.INPUT_DATA,
                                data_key="input1"
                            ),
                        ),
                        serialized_data="1",
                        serialized_type="int",
                        serialized_expected_type="int",
                    )
                ]
            },
        ]
        subtrees = [TreeData(**fields) for fields in subtree_fields]

        subtree_manager_subtree_data = (
            subtree_manager.get_subtree_data()
        )
        assert subtree_manager_subtree_data == []

        for i, tree in enumerate(subtrees):
            subtree_manager.add_subtree_data("node_" + str(i), tree)

        subtree_manager_subtree_data = (
            subtree_manager.get_subtree_data()
        )
        assert len(subtree_manager_subtree_data) == len(subtrees)
        for subtree_manager_tree, tree, fields in zip(
            subtree_manager_subtree_data, subtrees, subtree_fields
        ):
            for field in fields.keys():
                assert getattr(subtree_manager_tree, field) == getattr(tree, field)

    def test_clear_subtrees(self, subtree_manager: SubtreeManager):
        subtree_manager.add_subtree_structure("node_name", TreeStructure())
        subtree_manager.add_subtree_state("node_name", TreeState())
        subtree_manager.add_subtree_data("node_name", TreeData())
        subtree_manager.add_subtree_structure("node_name_2", TreeStructure())
        subtree_manager.add_subtree_state("node_name_2", TreeState())
        subtree_manager.add_subtree_data("node_name_2", TreeData())
        subtree_manager_subtree_structures = (
            subtree_manager.get_subtree_structures()
        )
        subtree_manager_subtree_states = (
            subtree_manager.get_subtree_states()
        )
        subtree_manager_subtree_data = (
            subtree_manager.get_subtree_data()
        )
        assert len(subtree_manager_subtree_structures) > 0
        assert len(subtree_manager_subtree_states) > 0
        assert len(subtree_manager_subtree_data) > 0
        subtree_manager.clear_subtrees()
        subtree_manager_subtree_structures = (
            subtree_manager.get_subtree_structures()
        )
        subtree_manager_subtree_states = (
            subtree_manager.get_subtree_states()
        )
        subtree_manager_subtree_data = (
            subtree_manager.get_subtree_data()
        )
        assert subtree_manager_subtree_structures == []
        assert subtree_manager_subtree_states == []
        assert subtree_manager_subtree_data == []

    def test_remove_subtree(self, subtree_manager: SubtreeManager):
        subtree_manager.add_subtree_structure("node_name", TreeStructure())
        subtree_manager.add_subtree_state("node_name", TreeState())
        subtree_manager.add_subtree_data("node_name", TreeData())
        subtree_manager.add_subtree_structure("node_name.nested", TreeStructure())
        subtree_manager.add_subtree_state("node_name.nested", TreeState())
        subtree_manager.add_subtree_data("node_name.nested", TreeData())
        subtree_manager.add_subtree_structure("node_name_2", TreeStructure())
        subtree_manager.add_subtree_state("node_name_2", TreeState())
        subtree_manager.add_subtree_data("node_name_2", TreeData())
        subtree_manager_subtree_structures = (
            subtree_manager.get_subtree_structures()
        )
        subtree_manager_subtree_states = (
            subtree_manager.get_subtree_states()
        )
        subtree_manager_subtree_data = (
            subtree_manager.get_subtree_data()
        )
        assert len(subtree_manager_subtree_structures) == 3
        assert len(subtree_manager_subtree_states) == 3
        assert len(subtree_manager_subtree_data) == 3
        subtree_manager.remove_subtree("node_name")
        subtree_manager_subtree_structures = (
            subtree_manager.get_subtree_structures()
        )
        subtree_manager_subtree_states = (
            subtree_manager.get_subtree_states()
        )
        subtree_manager_subtree_data = (
            subtree_manager.get_subtree_data()
        )
        assert len(subtree_manager_subtree_structures) == 1
        assert len(subtree_manager_subtree_states) == 1
        assert len(subtree_manager_subtree_data) == 1

    def test_check_publish_subtrees_false(
        self, subtree_manager_no_publish: SubtreeManager
    ):
        assert not subtree_manager_no_publish.get_publish_subtrees()

    def test_check_publish_subtrees_true(self, subtree_manager: SubtreeManager):
        assert subtree_manager.get_publish_subtrees()

