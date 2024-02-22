import pytest
from ros_bt_py.debug_manager import DebugManager
from ros_bt_py_interfaces.msg import Node, Tree
from ros_bt_py.exceptions import BehaviorTreeException


class TestDebugManager:
    @pytest.fixture
    def debug_manager_no_publish(self):
        node = Node()
        debug_manager = DebugManager(node)
        return debug_manager

    @pytest.fixture
    def debug_manager(self):
        node = Node()
        debug_manager = DebugManager(node)
        debug_manager.set_execution_mode(
            single_step=False,
            collect_performance_data=False,
            publish_subtrees=True,
            collect_node_diagnostics=False,
        )
        return debug_manager

    def test_add_subtree_info(self, debug_manager):
        subtree_fields = [
            {
                "name": "subtree_0",
                "tick_frequency_hz": 42.5,
                "state": Tree.IDLE,
                "nodes": [
                    Node(
                        name="node_0.0",
                        state=Node.IDLE,
                        child_names=["node_0.0.0", "node_0.0.1"],
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
                        state=Node.RUNNING,
                        child_names=["node_1.0.0", "node_1.0.1"],
                    )
                ],
            },
        ]
        subtrees = [Tree(**fields) for fields in subtree_fields]

        debug_manager_subtree_states = debug_manager.get_debug_info_msg().subtree_states
        assert debug_manager_subtree_states == []

        for i, tree in enumerate(subtrees):
            debug_manager.add_subtree_info("node_" + str(i), tree)

        debug_manager_subtree_states = debug_manager.get_debug_info_msg().subtree_states
        assert len(debug_manager_subtree_states) == len(subtrees)
        for debug_manager_tree, tree, fields in zip(
            debug_manager_subtree_states, subtrees, subtree_fields
        ):
            for field in fields.keys():
                assert getattr(debug_manager_tree, field) == getattr(tree, field)

    def test_clear_subtrees(self, debug_manager):
        debug_manager.add_subtree_info("node_name", Tree())
        debug_manager_subtree_states = debug_manager.get_debug_info_msg().subtree_states
        assert len(debug_manager_subtree_states) > 0
        debug_manager.clear_subtrees()
        debug_manager_subtree_states = debug_manager.get_debug_info_msg().subtree_states
        assert debug_manager_subtree_states == []

    def test_check_publish_subtrees_false(self, debug_manager_no_publish: DebugManager):
        assert debug_manager_no_publish.get_publish_subtrees() == False

    def test_check_publish_subtrees_true(self, debug_manager: DebugManager):
        assert debug_manager.get_publish_subtrees() == True

    def test_add_subtree_info_exception(self, debug_manager_no_publish: DebugManager):
        with pytest.raises(BehaviorTreeException):
            debug_manager_no_publish.add_subtree_info("node_name", Tree())
