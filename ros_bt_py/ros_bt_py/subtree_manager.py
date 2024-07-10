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

from ros_bt_py_interfaces.msg import Tree
from ros_bt_py.exceptions import BehaviorTreeException
from ros_bt_py_interfaces.msg import SubtreeInfo


@typechecked
class SubtreeManager(object):
    """
    Manages the collection of the states of a :class:`ros_bt_py.nodes.Subtree`.

    The subtree states are published by the :class:`TreeManager`.
    """

    def __init__(self):

        self.subtrees: Dict[str, Tree] = {}
        self._publish_subtrees: bool = False

        self._lock = Lock()
        with self._lock:
            self._subtree_info_msg = SubtreeInfo()

    def set_publish_subtrees(
        self,
        publish_subtrees: bool,
    ) -> None:
        self._publish_subtrees = publish_subtrees

    def get_subtree_info_msg(self) -> SubtreeInfo:
        with self._lock:
            return deepcopy(self._subtree_info_msg)

    def add_subtree_info(self, node_name: str, subtree_msg: Tree):
        """
        Publish subtree information.

        Used by the :class:`ros_bt_py.nodes.Subtree`.

        Serialization of subtrees (and calling this method) should
        only happen when the `publish_subtrees` option is set via
        `set_publish_subtrees` in :class:`TreeManager`.

        :param str node_name:

        The name of the subtree node. This will be prefixed to the
        subtree name to ensure it is unique.

        :raises: `ros_bt_py.exceptions.BehaviorTreeException`

        If this method is called when `publish_subtrees` is `False`.
        """
        subtree_name = f"{node_name}"
        with self._lock:
            if not self._publish_subtrees:
                raise BehaviorTreeException(
                    "Trying to add subtree info when subtree publishing is disabled!"
                )
            self.subtrees[subtree_name] = subtree_msg
            self._subtree_info_msg.subtree_states = list(self.subtrees.values())

    def clear_subtrees(self) -> None:
        with self._lock:
            self.subtrees.clear()
            self._subtree_info_msg.subtree_states = []

    def get_publish_subtrees(self) -> bool:
        with self._lock:
            return self._publish_subtrees
