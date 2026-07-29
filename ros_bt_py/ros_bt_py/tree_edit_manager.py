# Copyright (c) 2026 FZI Forschungszentrum Informatik
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
#    * Neither the name of the copyright holder nor the names of its
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

import uuid
from copy import deepcopy
from typing import Any

from ros_bt_py.tree_exec_manager import TreeExecManager, is_edit_service
from ros_bt_py.vendor.result import Err, Ok, Result

from typeguard import typechecked

from ros_bt_py_interfaces.msg import NodeIO

from ros_bt_py_interfaces.srv import (
    AddNode,
    AddNodeAtIndex,
    ChangeTreeName,
    GenerateSubtree,
    GetSubtree,
    LoadTree,
    MorphNode,
    MoveNode,
    RemoveNode,
    ReplaceNode,
    SetOptions,
    WireNodeData,
)

from ros_bt_py.debug_manager import DebugManager

from ros_bt_py.ros_helpers import ros_to_uuid, uuid_to_ros, wiring_has_id


class TreeEditManager(TreeExecManager):
    """
    Provide methods to edit a Behavior Tree
      in addition to the inherited load and run functions.

    These methods are suited (intended, even) for use as ROS service handlers.
    """

    @typechecked
    def find_nodes_in_cycles(self) -> list[uuid.UUID]:
        """Return a list of all nodes in the tree that are part of cycles."""
        safe_node_ids: list[uuid.UUID] = []
        nodes_in_cycles: list[uuid.UUID] = []
        # Follow the chain of parent nodes for each node name in self.nodes
        for starting_id in self.nodes.keys():
            cycle_candidates = [starting_id]
            current_node = self.nodes[starting_id]
            while current_node.parent:
                current_node = self.nodes[current_node.parent.node_id]
                cycle_candidates.append(current_node.node_id)
                if current_node.node_id == starting_id:
                    nodes_in_cycles.extend(cycle_candidates)
                    break
                if current_node.node_id in safe_node_ids:
                    # We've already checked for cycles from the parent node, no
                    # need to do that again.
                    safe_node_ids.extend(cycle_candidates)
                    break
            if not current_node.parent:
                safe_node_ids.extend(cycle_candidates)

        return nodes_in_cycles

    ####################
    # Service Handlers #
    ####################

    @is_edit_service
    @typechecked
    def add_node(
        self, request: AddNode.Request, response: AddNode.Response
    ) -> AddNode.Response:
        """
        Add the node in this request to the tree.

        :param ros_bt_py_msgs.srv.AddNodeRequest request:
          A request describing the node to add.
        """
        internal_request = AddNodeAtIndex.Request(
            parent_node_id=request.parent_node_id,
            node=request.node,
            new_child_index=-1,
        )
        internal_response = AddNodeAtIndex.Response()
        internal_response = self.add_node_at_index(
            request=internal_request, response=internal_response
        )

        response.success = internal_response.success
        response.error_message = internal_response.error_message
        return response

    @is_edit_service
    @typechecked
    def add_node_at_index(
        self, request: AddNodeAtIndex.Request, response: AddNodeAtIndex.Response
    ) -> AddNodeAtIndex.Response:
        """
        Add the node in this request to the tree.

        The `node_id` from the request is discarded and a new one is randomly generated.
        The actual is that the node is assigned is included in the response.

        :param ros_bt_py_msgs.srv.AddNodeAtIndexRequest request:
            A request describing the node to add.
        """
        node_msg = request.node
        node_msg.node_id = uuid_to_ros(uuid.uuid4())
        match self.instantiate_node_from_msg(
            node_msg=request.node,
            ros_node=self.ros_node,
        ):
            case Err(e):
                response.success = False
                response.error_message = str(e)
                return response
            case Ok(n):
                instance = n

        response.success = True
        response.node_id = node_msg.node_id

        match ros_to_uuid(request.parent_node_id):
            case Err(e):
                response.success = False
                response.error_message = e
                return response
            case Ok(p_id):
                parent_node_id = p_id

        # Add node as child of the named parent, if any
        if parent_node_id != uuid.UUID(int=0):
            if parent_node_id not in self.nodes:
                response.success = False
                response.error_message = (
                    f"Parent {request.parent_node_id} of node "
                    f"{instance.name} does not exist!"
                )
                # Remove node from tree
                self.remove_node(
                    request=RemoveNode.Request(
                        node_name=instance.name, remove_children=False
                    ),
                    response=RemoveNode.Response(),
                )
                return response

            match self.nodes[parent_node_id].add_child(
                child=instance, at_index=request.new_child_index
            ):
                case Err(e):
                    response.success = False
                    response.error_message = str(e)
                    return response
                case Ok(_):
                    pass

        # Add children from msg to node
        missing_children = []
        for child_id in request.node.child_ids:
            if child_id in self.nodes:
                match instance.add_child(self.nodes[child_id]):
                    case Err(e):
                        self.get_logger().warn(f"Could not add child: {str(e)}")
                        missing_children.append(child_id)
                    case Ok(_):
                        pass
            else:
                missing_children.append(child_id)
        if missing_children:
            response.success = False
            response.error_message = (
                f"Children for node {instance.name} are not or could not be added"
                f"in tree: {str(missing_children)}"
            )
            # Remove node from tree to restore state before insertion attempt
            self.remove_node(
                request=RemoveNode.Request(
                    node_name=instance.name, remove_children=False
                ),
                response=RemoveNode.Response(),
            )

        nodes_in_cycles = self.find_nodes_in_cycles()
        if nodes_in_cycles:
            response.success = False
            response.error_message = (
                f"Found cycles in tree {self.name} after inserting node "
                f"{request.node.name}. Nodes in cycles: {str(nodes_in_cycles)}"
            )
            # First, remove all of the node's children to avoid infinite
            # recursion in remove_node()
            for child_id in [c.node_id for c in instance.children]:
                match instance.remove_child(child_id):
                    case Err(_):
                        return response
                    case Ok(_):
                        pass

            # Then remove the node from the tree
            self.remove_node(
                request=RemoveNode.Request(
                    node_name=instance.name, remove_children=False
                ),
                response=RemoveNode.Response(),
            )
            return response

        return response

    @is_edit_service
    @typechecked
    def change_tree_name(
        self, request: ChangeTreeName.Request, response: ChangeTreeName.Response
    ) -> ChangeTreeName.Response:
        """Change the name of the currently loaded tree."""
        self.name = request.name

        response.success = True

        return response

    @is_edit_service
    @typechecked
    def remove_node(
        self, request: RemoveNode.Request, response: RemoveNode.Response
    ) -> RemoveNode.Response:
        """
        Remove the node identified by `request.node_name` from the tree.

        If the parent of the node removed supports enough children to
        take on all of the removed node's children, it will. Otherwise,
        children will be orphaned.
        """
        match ros_to_uuid(request.node_id):
            case Err(e):
                response.success = False
                response.error_message = e
                return response
            case Ok(n_id):
                node_id = n_id

        if node_id not in self.nodes:
            response.success = False
            response.error_message = (
                f"No node with id {request.node_id} in tree {self.name}"
            )
            return response

        target_node = self.nodes[node_id]

        node_ids_to_remove = {target_node.node_id}
        if request.remove_children:
            for child in target_node.get_children_recursive():
                node_ids_to_remove.add(child.node_id)

        # Unwire wirings that have removed nodes as source or target
        unwire_response = self.unwire_data(
            request=WireNodeData.Request(
                wirings=[
                    wiring
                    for wiring in self._tree_structure.data_wirings
                    if any(
                        [
                            wiring_has_id(wiring, node_id)
                            for node_id in node_ids_to_remove
                        ]
                    )
                ]
            ),
            response=WireNodeData.Response(),
        )
        if not unwire_response.success:
            response.success = False
            response.error_message = (
                f"Failed to unwire nodes for removal: {unwire_response.error_message}"
            )
            return response

        # Remove nodes in the reverse order they were added to the
        # list, i.e. the "deepest" ones first. This ensures that the
        # parent we refer to in the error message still exists.

        for n_id in reversed(list(node_ids_to_remove)):
            r_node = self.nodes[n_id]

            # If we have a parent, remove the node from that parent
            # TODO Why this convoluted double lookup, the `parent` reference should be good?
            if (
                r_node.parent is not None and r_node.parent.node_id in self.nodes  # type: ignore
            ):
                self.nodes[r_node.parent.node_id].remove_child(r_node.node_id)  # type: ignore
            del self.nodes[r_node.node_id]

        # This is moved down here, because setting the parent to None breaks the unwire
        if not request.remove_children:
            # If we're not removing the children, at least set their parent to None
            for child in target_node.children:
                child.parent = None

        for n_id in node_ids_to_remove:
            self.subtree_manager.remove_subtree(n_id)

        response.success = True
        return response

    @is_edit_service
    @typechecked
    def morph_node(
        self, request: MorphNode.Request, response: MorphNode.Response
    ) -> MorphNode.Response:
        """Morphs the flow control node into the new node provided in `request.new_node`."""
        match ros_to_uuid(request.node_id):
            case Err(e):
                response.success = False
                response.error_message = e
                return response
            case Ok(n_id):
                node_id = n_id

        if node_id not in self.nodes:
            response.success = False
            response.error_message = f"No node with id {node_id} in tree {self.name}"
            return response

        old_node = self.nodes[node_id]
        request.new_node.node_id = uuid_to_ros(node_id)

        match self.instantiate_node_from_msg(request.new_node, self.ros_node):
            case Err(e):
                response.success = False
                response.error_message = f"Error instantiating node {str(e)}"
                return response
            case Ok(n):
                new_node = n

        # First unwire all data connection to the existing node
        wire_request = WireNodeData.Request(
            wirings=[
                wiring
                for wiring in self._tree_structure.data_wirings
                if wiring_has_id(wiring, old_node.node_id)
            ],
            ignore_failure=False,
        )
        unwire_resp = self.unwire_data(
            request=wire_request, response=WireNodeData.Response()
        )
        if not get_success(unwire_resp):
            return MorphNode.Response(
                success=False,
                error_message=(
                    f"Failed to unwire data for node {old_node.name}: "
                    f"{get_error_message(unwire_resp)}"
                ),
            )
        # Re-wirings are always best effort, any failures are ignored
        wire_request.ignore_failure = True

        parent = None
        if old_node.parent:
            parent = old_node.parent
            # Remember the old index so we can insert the new instance at
            # the same position
            old_child_index = parent.get_child_index(old_node.node_id)

            if old_child_index is None:
                return MorphNode.Response(
                    success=False,
                    error_message=(
                        f"Parent of node {old_node.name} claims to have no child with that name?!"
                    ),
                )
            match parent.remove_child(old_node.node_id):
                case Err(e):
                    response.success = False
                    response.error_message = (
                        f"Could not remove child from parent: {str(e)}"
                    )
                    return response
                case Ok(_):
                    pass
            match parent.add_child(new_node, at_index=old_child_index):
                case Err(e):
                    parent.add_child(old_node, at_index=old_child_index)
                    self.wire_data(
                        request=wire_request, response=WireNodeData.Response()
                    )
                    response.error_message = (
                        f"Failed to add new instance of node {old_node.name}: {str(e)}"
                    )
                    response.success = False
                    return response
                case Ok(_):
                    pass

        # Move the children from old to new
        for child_id in [child.node_id for child in old_node.children]:
            match (
                old_node.remove_child(child_id)
                .map_err(
                    lambda err: f"Could not remove child from old node: {str(err)}"
                )
                .and_then(lambda child: new_node.add_child(child))
                .map_err(lambda err: f"Could not add child to new node: {str(err)}")
            ):
                case Err(e):
                    response.success = False
                    response.error_message = e
                    return response
                case Ok(_):
                    pass

        # Add the new node to self.nodes
        del self.nodes[old_node.node_id]
        self.nodes[new_node.node_id] = new_node

        rewire_resp = self.wire_data(
            request=wire_request, response=WireNodeData.Response()
        )
        if not get_success(rewire_resp):
            response.error_message = (
                f"Failed to re-wire data to new node {new_node.name}:"
                f" {get_error_message(rewire_resp)}"
            )
            response.success = False
            return response

        response.success = True
        return response

    @is_edit_service
    @typechecked
    def set_options(  # noqa: C901
        self, request: SetOptions.Request, response: SetOptions.Response
    ) -> SetOptions.Response:
        """
        Set the option values of a given node.

        This is an "edit service", i.e. it can only be used when the
        tree has not yet been initialized or has been shut down.
        """
        match ros_to_uuid(request.node_id):
            case Err(e):
                response.success = False
                response.error_message = e
                return response
            case Ok(n_id):
                node_id = n_id

        if node_id not in self.nodes:
            response.success = False
            response.error_message = (
                f"Unable to find node {node_id} in tree {self.name}"
            )
            return response

        node = self.nodes[node_id]

        # Because options are used at construction time, we need to
        # construct a new node with the new options.

        # First, we need to add the option values that didn't change
        # to our dict:
        new_inputs = {io.key: io for io in request.inputs}
        for key, container in node.node_config.inputs.items():
            if key in new_inputs.keys():
                continue
            new_inputs[key] = NodeIO(
                key=key,
                type=container.serialize_type(),
                serialized_value=container.serialize_value(),
            )

        # Now we can construct the new node - no need to call setup,
        # since we're guaranteed to be in the edit state
        # (i.e. `root.setup()` will be called before anything that
        # needs the node to be set up)
        node_msg = node.to_structure_msg()
        node_msg.inputs = new_inputs.values()
        if request.rename_node:
            node_msg.name = request.new_name
        match self.instantiate_node_from_msg(node_msg, ros_node=self.ros_node):
            case Err(e):
                response.success = False
                response.error_message = str(e)
                return response
            case Ok(n):
                new_node = n

        # Use this request to unwire any data connections the existing
        # node has - if we didn't do this, the node wouldn't ever be
        # garbage collected, among other problems.
        #
        # We'll use the same request to re-wire the connections to the
        # new node (or the old one, if anything goes wrong).
        wire_request = WireNodeData.Request(
            wirings=[
                wiring
                for wiring in self._tree_structure.data_wirings
                if wiring_has_id(wiring, node_id)
            ],
            ignore_failure=False,
        )
        unwire_resp = self.unwire_data(
            request=wire_request, response=WireNodeData.Response()
        )
        if not get_success(unwire_resp):
            response.success = False
            response.error_message = (
                f"Failed to unwire data for node {node.name}: "
                f"{get_error_message(unwire_resp)}"
            )
            return response
        # Re-wirings are always best effort, any failures are silently ignored.
        wire_request.ignore_failure = True

        parent = None
        if node.parent:
            parent = node.parent
            # Remember the old index so we can insert the new instance at
            # the same position
            old_child_index = parent.get_child_index(node.node_id)

            if old_child_index is None:
                response.success = False
                response.error_message = (
                    f"Parent of node {node.name} claims to "
                    f"have no child with that name?!"
                )
                return response

            match parent.remove_child(node.node_id):
                case Err(e):
                    error_message = (
                        f"Failed to remove old instance of node {node.name}: {str(e)}"
                    )
                    wire_request.ignore_failure = True
                    self.wire_data(
                        request=wire_request, response=WireNodeData.Response()
                    )
                    response.success = False
                    response.error_message = error_message
                    return response
                case Ok(_):
                    pass

            match parent.add_child(new_node, at_index=old_child_index):
                case Err(e):
                    parent.add_child(node, at_index=old_child_index)
                    self.wire_data(
                        request=wire_request, response=WireNodeData.Response()
                    )
                    response.success = False
                    response.error_message = (
                        f"Failed to add new instance of node {node.name}: {str(e)}"
                    )
                    return response
                case Ok(_):
                    pass

        # Add the new node to self.nodes
        self.nodes[node_id] = new_node

        self.wire_data(request=wire_request, response=WireNodeData.Response())

        # This line is important: The list comprehension creates a
        # new list that won't be affected by calling
        # remove_child()!
        for child_id in [child.node_id for child in node.children]:
            match node.remove_child(child_id).and_then(
                lambda child: new_node.add_child(child)
            ):
                case Err(e):
                    response.success = False
                    response.error_message = (
                        f"Failed to transfer children to new node: {str(e)}"
                    )
                    return response
                case Ok(_):
                    pass

        # We made it!
        response.success = True
        return response

    @is_edit_service
    @typechecked
    def move_node(
        self, request: MoveNode.Request, response: MoveNode.Response
    ) -> MoveNode.Response:
        """Move the named node to a different parent and insert it at the given index."""
        match ros_to_uuid(request.node_id):
            case Err(e):
                response.success = False
                response.error_message = e
                return response
            case Ok(n_id):
                node_id = n_id

        match ros_to_uuid(request.parent_node_id):
            case Err(e):
                response.success = False
                response.error_message = e
                return response
            case Ok(p_id):
                parent_node_id = p_id

        if node_id not in self.nodes:
            response.success = False
            response.error_message = f'Node to be moved ("{node_id}") is not in tree.'
            return response

        node = self.nodes[node_id]

        # Empty parent name -> just remove node from parent
        if parent_node_id == uuid.UUID(int=0):
            if node.parent is not None:
                match node.parent.remove_child(node.node_id):
                    case Err(e):
                        response.success = False
                        response.error_message = (
                            f"Could not remove child {node.name} "
                            f"from {node.parent.name}: {str(e)}"
                        )
                        return response
                    case Ok(_):
                        pass

            response.success = True
            return response

        if parent_node_id not in self.nodes:
            response.success = False
            response.error_message = f'New parent ("{parent_node_id}") is not in tree.'
            return response

        new_parent = self.nodes[parent_node_id]

        if (
            new_parent.node_config.max_children is not None
            and len(new_parent.children) == new_parent.node_config.max_children
        ):
            response.success = False
            response.error_message = (
                f"Cannot move node {node.name} to new parent node {new_parent.name}. "
                "Parent node already has the maximum number "
                f"of children ({new_parent.node_config.max_children})."
            )
            return response

        # If the new parent is part of the moved node's subtree, we'd
        # get a cycle, so check for that and fail if true!
        match node.get_subtree_msg():
            case Err(e):
                response.success = False
                response.error_message = (
                    f"Could not generate subtree msg from {node.name}: {str(e)}"
                )
                return response
            case Ok(s_m):
                subtree_msg = s_m
        if new_parent.node_id in [
            subtree_node.name for subtree_node in subtree_msg[0].nodes
        ]:
            response.success = False
            response.error_message = (
                f"Cannot move node {node.name} to new parent node {new_parent.name}. "
                f"{new_parent.name} is a child of {node.name}!"
            )
            return response

        # Remove node from old parent, if any
        old_parent = node.parent
        if old_parent is not None:
            match old_parent.remove_child(node.node_id):
                case Err(e):
                    response.success = False
                    response.error_message = (
                        f"Failed to remove child {node.node_id} "
                        f"from {old_parent.name}: {str(e)}"
                    )
                    return response

        # Add node to new parent
        match new_parent.add_child(child=node, at_index=request.new_child_index):
            case Err(e):
                response.success = False
                response.error_message = (
                    f"Failed to add child {node.name} to {new_parent.name}: {str(e)}"
                )
                return response
            case Ok(_):
                pass

        response.success = True
        return response

    @is_edit_service
    @typechecked
    def replace_node(
        self, request: ReplaceNode.Request, response: ReplaceNode.Response
    ) -> ReplaceNode.Response:
        """
        Replace the named node with `new_node`.

        Will also move all children of the old node to the new one, but
        only if `new_node` supports that number of children. Otherwise,
        this will return an error and leave the tree unchanged.
        """
        match ros_to_uuid(request.old_node_id):
            case Err(e):
                response.success = False
                response.error_message = e
                return response
            case Ok(n_id):
                old_node_id = n_id

        match ros_to_uuid(request.new_node_id):
            case Err(e):
                response.success = False
                response.error_message = e
                return response
            case Ok(n_id):
                new_node_id = n_id

        if old_node_id not in self.nodes:
            response.success = False
            response.error_message = (
                f'Node to be replaced ("{old_node_id}")is not in tree.'
            )
            return response
        if new_node_id not in self.nodes:
            response.success = False
            response.error_message = (
                f'Replacement node ("{new_node_id}") is not in tree.'
            )
            return response

        old_node = self.nodes[old_node_id]
        new_node = self.nodes[new_node_id]

        if (
            new_node.node_config.max_children is not None
            and len(old_node.children) > new_node.node_config.max_children
        ):
            response.success = False
            response.error_message = (
                f'Replacement node ("{new_node.name}") does not support the number of'
                f"children required ({old_node.name} has "
                f"{len(old_node.children)} children, "
                f"{new_node.name} supports {new_node.node_config.max_children}."
            )
            return response

        # TODO(nberg): Actually implement this

        # If the new node has inputs/outputs with the same name and
        # type as the old one,wire them the same way the old node was
        # wired

        # Note the old node's position in its parent's children array
        old_node_parent = old_node.parent
        # Initialize to 0 just to be sure. We *should* be guaranteed
        # to find the old node in its parent's children array, but
        # better safe than sorry.
        old_node_child_index = 0
        if old_node_parent is not None:
            for index, child in enumerate(old_node_parent.children):
                if child.node_id == old_node.node_id:
                    old_node_child_index = index
                    break

        # If it has the same parent as the old node, check its index, too.
        #
        # If the new node's index is smaller than the old one's, we
        # need to subtract one from old_node_child_index. Imagine we
        # want to replace B with A, and both are children of the same
        # node:
        #
        # parent.children = [A, B, C]
        #
        # Then old_node_child_index would be 1. But if we remove B
        #
        # parent.children = [A, C]
        #
        # And then move A to old_node_child_index, we end up with
        #
        # parent.children = [C, A]
        #
        # Which is wrong!
        if (
            new_node.parent is not None
            and old_node_parent is not None
            and new_node.parent.node_id == old_node_parent.node_id
            and old_node_child_index > 0
        ):
            for index, child in enumerate(new_node.parent.children):
                if child.node_id == new_node.node_id:
                    if index < old_node_child_index:
                        old_node_child_index -= 1
                    break

        # Move the children from old to new
        for child_id in [child.node_id for child in old_node.children]:
            match old_node.remove_child(child_id):
                case Err(e):
                    response.success = False
                    response.error_message = (
                        f"Could not remove child node {child_id}: {str(e)}"
                    )
                    return response
                case Ok(n):
                    child = n
            if child_id == new_node.node_id:
                continue
            match new_node.add_child(child):
                case Err(e):
                    response.success = False
                    response.error_message = (
                        f"Failed to add child {child.name}: {str(e)}"
                    )
                    return response
                case Ok(_):
                    pass

        # Remove the old node (we just moved the children, so we can
        # set remove_children to True)
        res = self.remove_node(
            request=RemoveNode.Request(
                node_id=uuid_to_ros(old_node.node_id), remove_children=True
            ),
            response=RemoveNode.Response(),
        )
        if not get_success(res):
            response.success = False
            response.error_message = (
                f'Could not remove old node: "{get_error_message(res)}"'
            )
            return response

        # Move the new node to the old node's parent (if it had one)
        if old_node_parent is not None:
            move_response = self.move_node(
                MoveNode.Request(
                    node_id=uuid_to_ros(new_node.node_id),
                    parent_node_id=uuid_to_ros(old_node_parent.node_id),
                    new_child_index=old_node_child_index,
                ),
                MoveNode.Response(),
            )
            if not move_response.success:
                response.success = move_response.success
                response.error_message = move_response.error_message
                return response

        response.success = True
        return response

    @is_edit_service
    @typechecked
    def wire_data(
        self, request: WireNodeData.Request, response: WireNodeData.Response
    ) -> WireNodeData.Response:
        """
        Connect the given pairs of node data to one another.

        :param ros_bt_py_msgs.srv.WireNodeDataRequest request:

        Contains a list of :class: `ros_bt_py_msgs.msg.NodeDataWiring`
        objects that model connections

        :returns: :class:`ros_bt_py_msgs.src.WireNodeDataResponse` or `None`
        """
        successful_wirings = []
        for wiring in request.wirings:
            match self.validate_wiring(wiring):
                case Err(e):
                    if request.ignore_failure:
                        continue
                    response.success = False
                    response.error_message = (
                        f'Failed to execute wiring "{wiring}": "{str(e)}'
                    )
                    return response
                case Ok(None):
                    successful_wirings.append(wiring)

        # only actually wire any data if there were no errors
        # We made it here, so all the Wirings should be valid. Time to save
        # them.
        self.wirings.extend(successful_wirings)
        response.success = True
        return response

    @is_edit_service
    @typechecked
    def unwire_data(
        self, request: WireNodeData.Request, response: WireNodeData.Response
    ) -> WireNodeData.Response:
        """
        Disconnect the given pairs of node data.

        :param ros_bt_py_msgs.srv.WireNodeDataRequest request:

        Contains a list of :class:`ros_bt_py_msgs.msg.NodeDataWiring`
        objects that model connections

        :returns: :class:`ros_bt_py_msgs.src.WireNodeDataResponse` or `None`
        """
        successful_unwirings = []
        for wiring in request.wirings:
            if wiring not in self.wirings:
                if request.ignore_failure:
                    continue
                response.success = False
                response.error_message = (
                    f'Failed to remove wiring "{wiring}": This wiring does not exist'
                )
                return response
            successful_unwirings.append(wiring)

        for wiring in successful_unwirings:
            self.wirings.remove(wiring)
        response.success = True
        return response

    @typechecked
    def get_subtree(
        self, request: GetSubtree.Request, response: GetSubtree.Response
    ) -> GetSubtree.Response:
        match ros_to_uuid(request.root_id):
            case Err(e):
                response.success = False
                response.error_message = e
                return response
            case Ok(n_id):
                root_id = n_id

        if root_id not in self.nodes:
            response.success = False
            response.error_message = f'Node "{request.root_id}" does not exist!'
            return response

        match self.nodes[root_id].get_subtree_msg():
            case Err(e):
                response.subtree = False
                response.error_message = (
                    f"Error retrieving subtree rooted at {request.root_id}: {str(e)}"
                )
                return response
            case Ok((s, _, _)):
                response.subtree = s
        response.success = True
        return response

    @typechecked
    def generate_subtree(
        self, request: GenerateSubtree.Request, response: GenerateSubtree.Response
    ) -> GenerateSubtree.Response:
        """
        Generate a subtree generated from the provided list of nodes and the loaded tree.

        This also adds all relevant parents to the tree message, resulting in a tree that is
        executable and does not contain any orpahned nodes.
        """
        whole_tree = deepcopy(self._tree_structure)

        match self.find_root():
            case Err(e):
                response.success = False
                response.error_message = f"Could not determine tree root: {str(e)}"
                return response
            case Ok(r):
                root = r

        if not root:
            response.success = False
            response.error_message = "No tree message available"
            return response
        nodes = set()
        for node in whole_tree.nodes:
            nodes.add(node.node_id)

        nodes_to_keep = set()
        nodes_to_remove = set()
        for node in whole_tree.nodes:
            for search_node in request.node_ids:
                if node.node_id == search_node or search_node in node.child_ids:
                    nodes_to_keep.add(node.node_id)
        # TODO We should recursively put parent nodes in `nodes_to_keep` to make
        # this easier to use (the request doesn't have to include all parents up to root).

        for node in nodes:
            if node not in nodes_to_keep:
                nodes_to_remove.add(node)

        manager = TreeEditManager(
            ros_node=self.ros_node,
            name="temporary_tree_manager",
            debug_manager=DebugManager(ros_node=self.ros_node),
        )

        load_response = LoadTree.Response()
        load_response = manager.load_tree(
            request=LoadTree.Request(tree=whole_tree),
            response=load_response,
        )

        if load_response.success:
            for node_id in nodes_to_remove:
                manager.remove_node(
                    RemoveNode.Request(node_id=node_id, remove_children=False),
                    RemoveNode.Response(),
                )
            match manager.find_root():
                case Err(e):
                    response.success = False
                    response.error_message = (
                        f"Could not determine new subtree root: {str(e)}"
                    )
                    return response
                case Ok(r):
                    root = r
            if root is None:
                self.get_logger().info("No nodes in tree")
            response.success = True
            response.tree = manager.structure_to_msg()
            return response
        else:
            response.success = False
            response.error_message = (
                "Could not load tree into the new subtree" + load_response.error_message
            )
            return response

    #########################
    # Service Handlers Done #
    #########################


@typechecked
def get_success(response: dict | Any) -> bool:
    if isinstance(response, dict):
        return response["success"]

    return response.success


@typechecked
def get_error_message(response: dict | Any) -> str:
    if isinstance(response, dict):
        return response["error_message"]

    return response.error_message
