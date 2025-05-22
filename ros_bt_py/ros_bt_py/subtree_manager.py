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
from typing import Dict

from typeguard import typechecked

from ros_bt_py_interfaces.msg import TreeStructure, TreeState, TreeData
from ros_bt_py.exceptions import BehaviorTreeException


@typechecked
class SubtreeManager(object):
    """
    Manages the collection of the states of a :class:`ros_bt_py.nodes.Subtree`.

    The subtree states are published by the :class:`TreeManager`.
    """

    def __init__(self):

        self.subtree_structures: Dict[str, TreeStructure] = {}
        self.subtree_states: Dict[str, TreeState] = {}
        self.subtree_data: Dict[str, TreeData] = {}
        self._publish_subtrees: bool = False
        self._publish_data: bool = False

        self._lock = Lock()

    def get_publish_subtrees(self) -> bool:
        with self._lock:
            return self._publish_subtrees

    def set_publish_subtrees(self, publish_subtrees: bool):
        with self._lock:
            self._publish_subtrees = publish_subtrees
        
    def get_publish_data(self) -> bool:
        with self._lock:
            return self._publish_data
        
    def set_publish_data(self, value: bool):
        with self._lock:
            self._publish_data = value

    def get_subtree_structures(self) -> list[TreeStructure]:
        with self._lock:
            if not self._publish_subtrees:
                return []
            return [ deepcopy(tree) for tree in self.subtree_structures.values() ]
        
    def get_subtree_states(self) -> list[TreeState]:
        with self._lock:
            if not self._publish_subtrees:
                return []
            return [ deepcopy(tree) for tree in self.subtree_states.values() ]
        
    def get_subtree_data(self) -> list[TreeData]:
        with self._lock:
            if not self._publish_subtrees:
                return []
            return [ deepcopy(tree) for tree in self.subtree_data.values() ]

    def add_subtree_structure(self, node_name: str, subtree_msg: TreeStructure):
        """
        Publish subtree information.

        Used by the :class:`ros_bt_py.nodes.Subtree`.

        :param str node_name:

        The name of the subtree node. This will be prefixed to the
        subtree name to ensure it is unique.
        """
        with self._lock:
            self.subtree_structures[node_name] = subtree_msg

    def add_subtree_state(self, node_name: str, subtree_msg: TreeState):
        with self._lock:
            self.subtree_states[node_name] = subtree_msg

    def add_subtree_data(self, node_name: str, subtree_msg: TreeData):
        with self._lock:
            self.subtree_data[node_name] = subtree_msg

    def clear_subtrees(self) -> None:
        with self._lock:
            self.subtree_structures.clear()
            self.subtree_states.clear()
            self.subtree_data.clear()

    def remove_subtree(self, node_name: str):

        def query_dict(node_name: str, dict: dict[str, object]):
            for tree_name in list(dict.keys()):
                if tree_name == node_name or tree_name.startswith(f"{node_name}."):
                    del dict[tree_name]

        with self._lock:
            query_dict(node_name, self.subtree_structures)
            query_dict(node_name, self.subtree_states)
            query_dict(node_name, self.subtree_data)

