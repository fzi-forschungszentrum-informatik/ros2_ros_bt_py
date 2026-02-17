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
from copy import deepcopy
from threading import Lock
from typing import Any, Dict, Mapping
import uuid

from result import Err, Ok, Result
from typeguard import typechecked

from ros_bt_py_interfaces.msg import TreeStructure, TreeState, TreeData


@typechecked
class SubtreeManager(object):
    """
    Collects information about all subtrees in a given tree.

    Subtrees are identified based on the node_id of the node that 'owns' the subtree,
      instead of the tree_id of the subtree itself.
    This helps with maintaining information when a node is updated because the node_id
      is persisted while the tree_id is not.
    """

    def __init__(self):

        self.subtree_structures: Dict[uuid.UUID, TreeStructure] = {}
        self.subtree_states: Dict[uuid.UUID, TreeState] = {}
        self.subtree_data: Dict[uuid.UUID, TreeData] = {}
        self.nested_subtree_managers: Dict[uuid.UUID, SubtreeManager] = {}
        self._publish_subtrees: bool = False
        self._publish_data: bool = False

        self._lock = Lock()

    def get_publish_subtrees(self) -> bool:
        with self._lock:
            return self._publish_subtrees

    @property
    def publish_subtrees(self) -> bool:
        return self._publish_subtrees

    @publish_subtrees.setter
    def publish_subtrees(self, publish_subtrees: bool) -> None:
        with self._lock:
            self._publish_subtrees = publish_subtrees
            for manager in self.nested_subtree_managers.values():
                manager.publish_subtrees = publish_subtrees

    def get_publish_data(self) -> bool:
        with self._lock:
            return self._publish_data

    def set_publish_data(self, value: bool):
        with self._lock:
            self._publish_data = value
            for manager in self.nested_subtree_managers.values():
                manager.set_publish_data(value)

    def get_subtree_structures(self) -> list[TreeStructure]:
        with self._lock:
            if not self._publish_subtrees:
                return []
            full_list = [deepcopy(tree) for tree in self.subtree_structures.values()]
            for manager in self.nested_subtree_managers.values():
                full_list.extend(manager.get_subtree_structures())
            return full_list

    def get_subtree_states(self) -> list[TreeState]:
        with self._lock:
            if not self._publish_subtrees:
                return []
            full_list = [deepcopy(tree) for tree in self.subtree_states.values()]
            for manager in self.nested_subtree_managers.values():
                full_list.extend(manager.get_subtree_states())
            return full_list

    def get_subtree_data(self) -> list[TreeData]:
        with self._lock:
            if not self._publish_subtrees:
                return []
            full_list = [deepcopy(tree) for tree in self.subtree_data.values()]
            for manager in self.nested_subtree_managers.values():
                full_list.extend(manager.get_subtree_data())
            return full_list

    def add_subtree_structure(
        self, node_id: uuid.UUID, subtree_msg: TreeStructure
    ) -> None:
        with self._lock:
            self.subtree_structures[node_id] = subtree_msg

    def add_subtree_state(self, node_id: uuid.UUID, subtree_msg: TreeState):
        with self._lock:
            self.subtree_states[node_id] = subtree_msg

    def add_subtree_data(self, node_id: uuid.UUID, subtree_msg: TreeData):
        with self._lock:
            self.subtree_data[node_id] = subtree_msg

    def add_nested_manager(self, node_id: uuid.UUID, subtree_manager: "SubtreeManager"):
        subtree_manager.publish_subtrees = self.publish_subtrees
        subtree_manager.set_publish_data(self.get_publish_data())
        with self._lock:
            self.nested_subtree_managers[node_id] = subtree_manager

    def clear_subtrees(self) -> None:
        with self._lock:
            self.subtree_structures.clear()
            self.subtree_states.clear()
            self.subtree_data.clear()
            self.nested_subtree_managers.clear()

    def remove_subtree(self, node_id: uuid.UUID):

        def query_dict(node_id: uuid.UUID, dict: dict[uuid.UUID, Any]):
            for n_id in list(dict.keys()):
                if n_id == node_id:
                    del dict[n_id]

        with self._lock:
            query_dict(node_id, self.subtree_structures)
            query_dict(node_id, self.subtree_states)
            query_dict(node_id, self.subtree_data)
            query_dict(node_id, self.nested_subtree_managers)
