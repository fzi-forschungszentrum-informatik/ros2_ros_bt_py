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

"""Module defining the Node class and helper functions representing a node in the behavior tree."""

from contextlib import contextmanager
import inspect
from types import ModuleType
from typeguard import typechecked

from ros_bt_py.vendor.result import Err, Ok, Result, do

import abc
import importlib
import re
import uuid
from typing import (
    Any,
    Callable,
    Iterator,
    Tuple,
    Optional,
    TypeVar,
)

import rclpy
import rclpy.logging
from rclpy.node import Node as ROSNode

from ros_bt_py_interfaces.msg import (
    NodeStructure,
    NodeState,
    NodeIO,
    Wiring,
    TreeStructure,
    UtilityBounds,
)

from ros_bt_py.data_flow_manager import DataFlowManager
from ros_bt_py.data_types import (
    DataContainer,
    ReferenceContainer,
    get_iotype_for_msg,
)
from ros_bt_py.debug_manager import DebugManager
from ros_bt_py.subtree_manager import SubtreeManager
from ros_bt_py.logging_manager import LoggingManager
from ros_bt_py.exceptions import (
    BehaviorTreeException,
    NodeStateError,
    NodeConfigError,
    TreeTopologyError,
)
from ros_bt_py.node_config import NodeConfig, NodeInputMap, NodeOutputMap
from ros_bt_py.helpers import BTNodeState
from ros_bt_py.ros_helpers import ros_to_uuid, uuid_to_ros


RET = TypeVar("RET")


class Node(abc.ABC):
    """
    Base class for Behavior Tree nodes.

    Each node has a set of inputs, outputs and options. At every tick
    (usually somewhere between 10 and 30 times a second),
    :meth:`tick` is called with the appropriate data.

    Nodes in a behavior Tree can be roughly divided into two classes,
    with two sub-classes each:

    Leaf Nodes
      These do not have any children and can take one of two forms:
      *Predicates* and *Behaviors*. *Predicates* check a condition and instantly
      return `SUCCEEDED` or `FAILED`. *Behaviors* are more involved and may
      return `RUNNING`, but should be interruptible (see :meth:`untick`).

    Inner Nodes
      These too come in two flavors: *Combiners* and *Decorators*. *Combiners*
      have multiple children and decide which of those children to run (and in
      what fashion) based on some criteria. *Decorators* however have only a
      single child and work with that child's result - for instance, a *Decorator*
      could invert `FAILED` into `SUCCEEDED`.
    """

    # Config template for each specific node class
    _node_config: NodeConfig

    @contextmanager
    def _dummy_report_state(self):
        self.logdebug("Reporting state up without debug manager")
        yield

    @contextmanager
    def _dummy_report_tick(self):
        self.logdebug("Ticking without debug manager")
        yield

    node_classes: dict[str, dict[str, type["Node"]]] = {}
    debug_manager: Optional[DebugManager]
    subtree_manager: Optional[SubtreeManager]
    logging_manager: Optional[LoggingManager]
    data_flow_manager: Optional[DataFlowManager]
    _state: BTNodeState

    def __init__(
        self,
        node_id: Optional[uuid.UUID] = None,
        name: Optional[str] = None,
        new_inputs: dict[str, DataContainer] = {},
        ros_node: Optional[ROSNode] = None,
        debug_manager: Optional[DebugManager] = None,
        subtree_manager: Optional[SubtreeManager] = None,
        logging_manager: Optional[LoggingManager] = None,
        data_flow_manager: Optional[DataFlowManager] = None,
    ) -> None:
        """
        Prepare class members.

        After this finishes, the Node is *not* ready to run. You still
        need to do your own initialization in :meth:`_do_setup`.

        Since the `__init__` method has to return None,
        it doesn't use `result` but raises errors as normal.
        The `from_msg` method catches any errors and passes them on as a `result.Err`.

        :param dict new_inputs: A map of string keys to `DataContainer` values.
        This can only replace existing values if they already exist,
        either statically or through `add_extra_in/outputs`.
        This is used to establish static/dynamic inputs and their values.

        :param debug_manager: Debug manager used to debug a behavior tree.

        :param name: Name of the node - defaults to None, in which case the node name
        is set to the name of its class.

        :raises: NodeConfigError

        If anything is wrong with the node configuration defined via
        :function:`ros_bt_py.node.define_bt_node`
        """
        if name is not None:
            self.name = name
        else:
            self.name = type(self).__name__
        if node_id is not None:
            self.node_id = node_id
        else:
            self.node_id = uuid.uuid4()

        self.tree_ref: Optional[uuid.UUID] = None

        # Only used to make finding the root of the tree easier
        self.parent: Optional[Node] = None
        self._state: BTNodeState = BTNodeState.UNINITIALIZED
        self.children: list[Node] = []

        self.subscriptions: list[Wiring] = []
        self.subscribers: list[tuple[Wiring, Callable[[type], None], type]] = []

        self._ros_node: Optional[ROSNode] = ros_node
        self.debug_manager = debug_manager
        self.subtree_manager = subtree_manager
        self.logging_manager = logging_manager
        self.data_flow_manager = data_flow_manager

        if not self._node_config:
            raise NodeConfigError("Missing node_config, cannot initialize!")

        # Copy the class NodeConfig so we can mess with it
        self.node_config = self._node_config.copy()
        # Set up data map wrappers, they still have access to inputs that are added later.
        self.inputs = NodeInputMap(f"{self.name}.inputs", self.node_config.inputs)
        self.outputs = NodeOutputMap(f"{self.name}.outputs", self.node_config.outputs)

        unset_inputs: dict[str, DataContainer] = {}
        for key, container in new_inputs.items():
            if key not in self.node_config.inputs:
                unset_inputs[key] = container
                continue
            if not self.node_config.inputs[key].is_compatible(container):
                raise NodeConfigError(
                    f"For key {key}, given input {container} is not compatible with "
                    f" the defined template {self.node_config.inputs[key]}"
                )
            self.node_config.inputs[key] = container

        match do(
            Ok(
                NodeConfig(
                    inputs=extra_inputs,
                    outputs=extra_outputs,
                    max_children=self.node_config.max_children,
                )
            )
            for extra_inputs in self.add_extra_inputs()
            for extra_outputs in self.add_extra_outputs()
        ).and_then(lambda config: self.node_config.extend(config)):
            case Err(e):
                raise e
            case Ok(None):
                pass

        for key, container in unset_inputs.items():
            if key not in self.node_config.inputs:
                raise NodeConfigError(
                    f"Input {key} is not available on this node."
                    f"Available inputs: {self.node_config.inputs.keys()}"
                )
            if not self.node_config.inputs[key].is_compatible(container):
                raise NodeConfigError(
                    f"For key {key}, given input {container} is not compatible with "
                    f" the defined template {self.node_config.inputs[key]}"
                )
            self.node_config.inputs[key] = container

        # Since we substituted some inputs with updated versions,
        # we have to update all references that were copied from the base config.
        # Do NOT update anything that was in `new_inputs` because that would overwrite values.
        for key, container in self.node_config.inputs.items():
            if key in new_inputs.keys():
                continue
            if not isinstance(container, ReferenceContainer):
                continue
            match container.set_type_map(self.node_config.inputs):
                case Err(e):
                    raise NodeConfigError(e)
                case Ok(None):
                    pass
        for container in self.node_config.outputs.values():
            if not isinstance(container, ReferenceContainer):
                continue
            match container.set_type_map(self.node_config.inputs):
                case Err(e):
                    raise NodeConfigError(e)
                case Ok(None):
                    pass

        # Don't setup automatically - nodes should be available as pure data
        # containers before the user decides to call setup() themselves!

    @property
    def state(self) -> BTNodeState:
        """State of the node."""
        return self._state

    @state.setter
    @typechecked
    def state(self, new_state: BTNodeState):
        self.logdebug(f"Setting state from  {self._state} to {new_state}")
        self._state = new_state

    @property
    def has_ros_node(self) -> bool:
        return self._ros_node is not None

    @property
    def ros_node(self) -> ROSNode:
        """
        Return the associated ROS node instance.

        If no instance is present an
        """
        if self._ros_node is not None:
            return self._ros_node
        else:
            error_msg = f"{self.name} does not have ROS node reference!"
            self.logerr(error_msg)
            raise RuntimeError(error_msg)

    @ros_node.setter
    @typechecked
    def ros_node(self, new_ros_node: ROSNode):
        self.logdebug(f"Setting new ROS node: {new_ros_node}")
        self._ros_node = new_ros_node

    def get_logger(self) -> Optional[LoggingManager]:
        return self.logging_manager

    def add_extra_inputs(self) -> Result[dict[str, DataContainer], NodeConfigError]:
        """
        Return a dictionary of extra inputs that you want to add to the node config.
        This can access all original inputs, including static values if they exist.
        """
        return Ok({})

    def add_extra_outputs(self) -> Result[dict[str, DataContainer], NodeConfigError]:
        """
        Return a dictionary of extra outputs that you want to add to the node config.
        This can access all original inputs, including static values if they exist.
        """
        return Ok({})

    @staticmethod
    def log_errors(
        func: Callable[["Node"], Result[RET, BehaviorTreeException]],
    ) -> Callable[["Node"], Result[RET, BehaviorTreeException]]:
        def inner(self: "Node") -> Result[RET, BehaviorTreeException]:
            match func(self):
                case Err(e):
                    self.logerr(str(e))
                    return Err(e)
                case Ok(v):
                    return Ok(v)

        return inner

    @log_errors
    def setup(self) -> Result[BTNodeState, BehaviorTreeException]:
        """
        Prepare the node to be ticked for the first time.

        This is called after all the input, output and option values
        have been registered (and in the case of options, populated), so
        you can use those values in your implementation of
        :meth:`_do_setup`

        Sets the state of the node to IDLE or BROKEN.

        :returns: Returns a result object with the new state or the error message.
        """
        report_state = self._dummy_report_state()
        if self.debug_manager:
            report_state = self.debug_manager.report_state(self, "SETUP")

        with report_state:
            if (
                self.state != BTNodeState.UNINITIALIZED
                and self.state != BTNodeState.SHUTDOWN
            ):
                return Err(
                    NodeStateError(
                        "Calling setup() is only allowed in states "
                        f"{BTNodeState.UNINITIALIZED} and {BTNodeState.SHUTDOWN}, "
                        f"but node {self.name} is in state {self.state}"
                    )
                )

            # Reset input/output reset state and set dynamic values to None
            for container in self.node_config.inputs.values():
                container.reset_value()
                container.flag_updated()

            for container in self.node_config.outputs.values():
                container.reset_value()
                container.reset_updated()

            match self._do_setup():
                case Err(e):
                    self.state = BTNodeState.BROKEN
                    return Err(e)
                case Ok(s):
                    self.state = s

        return Ok(self.state)

    @abc.abstractmethod
    @typechecked
    def _do_setup(self) -> Result[BTNodeState, BehaviorTreeException]:
        """
        Use this to do custom node setup.

        Note that this will be called once, when the tree is first
        started, before the first call of :meth:`tick`.
        """
        msg = f"Trying to setup a node of type {self.__class__.__name__}"
        "without _do_setup function!"

        self.logerr(msg)
        return Err(BehaviorTreeException(msg))

    @log_errors
    def tick(self) -> Result[BTNodeState, BehaviorTreeException]:
        """
        Handle node on tick action everytime this is called (at ~10-20Hz, usually).

        You should not need to override this method, but instead
        implement :meth:`_do_tick` in your own class.

        :returns:
          The state of the node after ticking - should be `SUCCEEDED`, `FAILED` or `RUNNING`.
        """
        report_tick = self._dummy_report_tick()
        if self.debug_manager:
            report_tick = self.debug_manager.report_tick(self)

        with report_tick:
            if self.state is BTNodeState.UNINITIALIZED:
                return Err(BehaviorTreeException("Trying to tick uninitialized node!"))

            # Check if any inputs are unset, if so return an error
            for key, container in self.node_config.inputs.items():
                if not container.has_value():
                    self.state = BTNodeState.BROKEN
                    return Err(BehaviorTreeException(f"Input for key {key} is unset."))

            # Outputs are updated in the tick. To catch that, we need to reset here.
            for container in self.node_config.outputs.values():
                container.reset_updated()

            match self._do_tick():
                case Err(e):
                    self.state = BTNodeState.BROKEN
                    return Err(e)
                case Ok(s):
                    self.state = s

            # Inputs are updated by other nodes' outputs, i.e. some time after
            # we use them here. In some cases, inputs might be connected to
            # child outputs (or even our own). If they are, update information
            # is lost, unless it is processed after all child ticks in the same
            # cycle!
            for container in self.node_config.inputs.values():
                container.reset_updated()

            match self.check_if_in_invalid_state(
                allowed_states=[
                    BTNodeState.RUNNING,
                    BTNodeState.SUCCEEDED,
                    BTNodeState.FAILED,
                    BTNodeState.ASSIGNED,
                    BTNodeState.UNASSIGNED,
                ],
                action_name="tick()",
            ):
                case Err(e):
                    return Err(e)
                case Ok(None):
                    pass

            if self.data_flow_manager is not None:
                match self.data_flow_manager.push_outputs(self.node_id):
                    case Err(e):
                        self.state = BTNodeState.BROKEN
                        return Err(BehaviorTreeException(e))
                    case Ok(None):
                        pass

            return Ok(self.state)

    @typechecked
    def check_if_in_invalid_state(
        self, allowed_states: list[BTNodeState], action_name: str
    ) -> Result[None, NodeStateError]:
        if self.state not in allowed_states:
            return Err(
                NodeStateError(
                    f"Node {self.name} ({type(self).__name__}) was in invalid state "
                    f"'{self.state}' after action {action_name}. "
                    f"Allowed states: {str(allowed_states)}"
                )
            )
        return Ok(None)

    @abc.abstractmethod
    @typechecked
    def _do_tick(self) -> Result[BTNodeState, BehaviorTreeException]:
        """
        Every Node class must override this.

        This method should **NOT** block, ever, and return one of the
        constants from `Node.Status`.

        :returns:
          One of the constants in :class:`ros_bt_py_msgs.msg.Node`
        """
        msg = f"Ticking a node of type {self.__class__.__name__} without _do_tick function!"
        self.logerr(msg)
        return Err(BehaviorTreeException(msg))

    @log_errors
    def untick(self) -> Result[BTNodeState, BehaviorTreeException]:
        """
        Signal a node that it should stop any background tasks.

        A new tick has started and this node has **not** been ticked.
        The node's state should be `IDLE` after calling this.

        The node's outputs' `updated` flags are also reset!

        A class inheriting from :class:`Node` should override :meth:`_do_untick` instead of this!
        """
        report_state = self._dummy_report_state()
        if self.debug_manager:
            report_state = self.debug_manager.report_state(self, "UNTICK")

        with report_state:
            if self.state is BTNodeState.UNINITIALIZED:
                return Err(
                    BehaviorTreeException("Trying to untick uninitialized node!")
                )
            match self._do_untick():
                case Err(e):
                    self.state = BTNodeState.BROKEN
                    return Err(e)
                case Ok(s):
                    self.state = s

            match self.check_if_in_invalid_state(
                allowed_states=[BTNodeState.IDLE, BTNodeState.PAUSED],
                action_name="untick()",
            ):
                case Err(e):
                    return Err(e)
                case Ok(None):
                    pass

            for container in self.node_config.outputs.values():
                container.reset_updated()

            return Ok(self.state)

    @abc.abstractmethod
    @typechecked
    def _do_untick(self) -> Result[BTNodeState, BehaviorTreeException]:
        """
        Abstract method used to implement the actual untick operations.

        This is called by :meth:`untick` - override it!

        After executing this method, your node should:

        1. Be in the IDLE or PAUSED state, unless an error happened
        2. Not execute any of its behavior in the background
        3. Be ready to resume on the next call of :meth:`tick`
        """
        msg = f"Unticking a node of type {self.__class__.__name__} without _do_untick function!"
        self.logerr(msg)
        return Err(BehaviorTreeException(msg))

    @log_errors
    def reset(self) -> Result[BTNodeState, BehaviorTreeException]:
        """
        Reset a node completly.

        Whereas :meth:`untick` / :meth:`_do_untick` only pauses
        execution, ready to be resumed, :meth:`reset` means returning
        to the same state the node was in right after calling :meth:`setup`

        """
        report_state = self._dummy_report_state()
        if self.debug_manager:
            report_state = self.debug_manager.report_state(self, "RESET")

        with report_state:
            if self.state is BTNodeState.UNINITIALIZED:
                return Err(BehaviorTreeException("Trying to reset uninitialized node!"))

            if self.state is BTNodeState.SHUTDOWN:
                return Err(BehaviorTreeException("Trying to reset shutdown node!"))

            # Reset input/output state and set outputs to None
            for container in self.node_config.inputs.values():
                container.flag_updated()

            for container in self.node_config.outputs.values():
                container.reset_value()
                container.reset_updated()

            match self._do_reset():
                case Err(e):
                    self.state = BTNodeState.BROKEN
                    return Err(e)
                case Ok(s):
                    self.state = s

            match self.check_if_in_invalid_state(
                allowed_states=[BTNodeState.IDLE], action_name="reset()"
            ):
                case Err(e):
                    return Err(e)
                case Ok(None):
                    pass

            return Ok(self.state)

    @abc.abstractmethod
    @typechecked
    def _do_reset(self) -> Result[BTNodeState, BehaviorTreeException]:
        """
        Abstract method used to implement the reset action.

        After executing this method, your node should:

        1. Be in the IDLE state
        2. Not be doing anything in the background
        3. On the next tick, behave as if it has just been created

        :returns:
          The new state of the node (should be IDLE unless an error happened)
        """
        msg = f"Resetting a node of type {self.__class__.__name__} without _do_reset function!"
        self.logerr(msg)
        return Err(BehaviorTreeException(msg))

    @log_errors
    def shutdown(self) -> Result[BTNodeState, BehaviorTreeException]:
        """
        Prepare a node for deletion.

        This method calls :meth:`_do_shutdown`, which any
        subclass must override.

        This method, in contrast to other similar methods,
        also descends down the whole tree and calls shutdown on all children.
        That means the `_do_shutdown` method is only responsible for the node itself.

        This gives the node a chance to clean up any resources it might
        be holding before getting deleted.

        :meth:`_do_shutdown` will not be called if the node has not been initialized yet.
        """
        report_state = self._dummy_report_state()
        if self.debug_manager:
            report_state = self.debug_manager.report_state(self, "SHUTDOWN")

        with report_state:
            error = None

            if self.state != BTNodeState.SHUTDOWN:
                match self._do_shutdown():
                    case Err(e):
                        self.state = BTNodeState.BROKEN
                        error = e
                    case Ok(s):
                        self.state = s

            for child in self.children:
                match child.shutdown():
                    case Err(e):
                        self.logwarn(
                            f"Node {child.name} raised the following error during shutdown"
                            f"Continuing to shutdown other nodes\n{e}"
                        )
                        error = e
                    case Ok(_):
                        pass

            unshutdown_children = [
                f"{child.name} ({type(child).__name__}), state: {child.state}"
                for child in self.children
                if child.state != BTNodeState.SHUTDOWN
            ]
            if len(unshutdown_children) > 0:
                self.logwarn(
                    "Not all children are shut down after calling shutdown(). "
                    f"List of not-shutdown children and states:\n{unshutdown_children}"
                )

            if error is not None:
                return Err(error)

            return Ok(self.state)

    @abc.abstractmethod
    @typechecked
    def _do_shutdown(self) -> Result[BTNodeState, BehaviorTreeException]:
        """
        Abstract method implementing the shutdown action.

        Implement this in your node class and release any resources you
        might be holding (file pointers, ROS topic subscriptions etc.)
        """
        msg = f"Shutting down a node of type {self.__class__.__name__}"
        "without _do_shutdown function!"

        self.logerr(msg)
        return Err(BehaviorTreeException(msg))

    def calculate_utility(self) -> Result[UtilityBounds, BehaviorTreeException]:
        """
        Calculate the utility bounds for this node.

        Unlike the other node functions, there is a default
        implementation for the corresponding method,
        :meth:`Node._do_calculate_utility()`.

        However, in order to get meaningful results, one should take
        care to use as many nodes as possible that provide their own
        implementation, since the default reports that there is no
        cost for execution.

        """
        return self._do_calculate_utility()

    # TODO Should this be flagged as abstract?
    def _do_calculate_utility(self) -> Result[UtilityBounds, BehaviorTreeException]:
        """
        Calculate utility values. This is a default implementation.

        :returns:

        A :class:`ros_bt_py_msgs.msg.UtilityBounds` message with
        `can_execute` set to `True`, all bounds set to 0.0 and all of
        the `has_bound` members set to `True`.

        That is, any node that does not override this method is
        considered to execute at no cost at all.

        """
        return Ok(
            UtilityBounds(
                can_execute=True,
                has_lower_bound_success=True,
                has_upper_bound_success=True,
                has_lower_bound_failure=True,
                has_upper_bound_failure=True,
            )
        )

    @typechecked
    def get_child_index(self, child_id: uuid.UUID) -> Optional[int]:
        """
        Get the index in the `children` array of the child with the given name.

        This is useful if you want to replace a child with another node.

        :returns:

        An integer index if a child with the given name exists, `None`
        if there's no such child

        """
        try:
            return [child.node_id for child in self.children].index(child_id)
        except ValueError:
            return None

    @typechecked
    def add_child(
        self, child: "Node", at_index: Optional[int] = None
    ) -> Result["Node", BehaviorTreeException | TreeTopologyError]:
        """Add a child to this node at the given index."""
        if (
            self.node_config.max_children is not None
            and len(self.children) == self.node_config.max_children
        ):
            error_msg = (
                "Trying to add child when maximum number of "
                "children (%d) is already present" % self.node_config.max_children
            )
            self.logerr(error_msg)
            return Err(BehaviorTreeException(error_msg))

        if child.node_id in (child1.node_id for child1 in self.children):
            return Err(
                TreeTopologyError(f"Already have a child with id '{child.node_id}'")
            )
        if at_index is None:
            at_index = len(self.children)

        if at_index < 0:
            at_index += len(self.children) + 1
        # Use array slicing to efficiently insert child at the correct position
        # (the value we assign needs to be a list for this to work)
        self.children[at_index:at_index] = [child]
        child.parent = self

        # return self to allow chaining of addChild calls
        return Ok(self)

    @typechecked
    def remove_child(self, child_id: uuid.UUID) -> Result["Node", KeyError]:
        """
        Remove the child with the given name and return it.

        :param basestring child_id: The uuid of the child to remove
        """
        child_index = self.get_child_index(child_id)
        if child_index is None:
            return Err(KeyError(f'Node {self.name} has no child with id "{child_id}"'))

        tmp = self.children[child_index]
        del self.children[child_index]
        tmp.parent = None
        return Ok(tmp)

    def __repr__(self) -> str:
        """Create a string representation of the node class."""
        return (
            f"{type(self).__name__}("
            f"name={self.name}, "
            f"id={self.node_id}), "
            f"parent_name:{self.parent.name if self.parent else ''}, "
            f"state: {self.state}, "
            f"inputs: {self.inputs}, "
            f"outputs: {self.outputs}, "
            f"children: {self.children}"
        )

    @typechecked
    def __eq__(self, other: Any) -> bool:
        """Check if all attributes of a node are equal."""
        return (
            self.node_id == other.node_id
            and self.name == other.name
            and self.parent == other.parent
            and self.state == other.state
            and type(self).__module__ == type(other).__module__
            and type(self).__name__ == type(other).__name__
            and self.inputs == other.inputs
            and self.outputs == other.outputs
            and self.children == other.children
        )

    @typechecked
    def __ne__(self, other: Any) -> bool:
        """Check if two nodes have a single differing attribute."""
        return not self == other

    # Logging methods - these add the name and id of the node
    #   to the associated logging_manager

    @typechecked
    def logdebug(self, message: str, stacklevel=3, internal=False) -> None:
        """
        Wrap call to the associated logging manager.

        Adds this node's name and type to the given message
        """
        logger = self.get_logger()
        if logger is not None:
            logger.debug(
                msg=message,
                node_id=self.node_id,
                node_name=self.name,
                stacklevel=stacklevel,
                internal=internal,
            )

    @typechecked
    def loginfo(self, message: str, stacklevel=3, internal=False) -> None:
        """
        Wrap call to the associated logging manager.

        Adds this node's name and type to the given message
        """
        logger = self.get_logger()
        if logger is not None:
            logger.info(
                msg=message,
                node_id=self.node_id,
                node_name=self.name,
                stacklevel=stacklevel,
                internal=internal,
            )

    @typechecked
    def logwarn(self, message: str, stacklevel=3, internal=False) -> None:
        """
        Wrap call to the associated logging manager.

        Adds this node's name and type to the given message
        """
        logger = self.get_logger()
        if logger is not None:
            logger.warn(
                msg=message,
                node_id=self.node_id,
                node_name=self.name,
                stacklevel=stacklevel,
                internal=internal,
            )

    @typechecked
    def logerr(self, message: str, stacklevel=3, internal=False) -> None:
        """
        Wrap call to the associated logging manager.

        Adds this node's name and type to the given message
        """
        logger = self.get_logger()
        if logger is not None:
            logger.error(
                msg=message,
                node_id=self.node_id,
                node_name=self.name,
                stacklevel=stacklevel,
                internal=internal,
            )

    @typechecked
    def logfatal(self, message: str, stacklevel=3, internal=False) -> None:
        """
        Wrap call to the associated logging manager.

        Adds this node's name and type to the given message
        """
        logger = self.get_logger()
        if logger is not None:
            logger.fatal(
                msg=message,
                node_id=self.node_id,
                node_name=self.name,
                stacklevel=stacklevel,
                internal=internal,
            )

    @classmethod
    @typechecked
    def from_msg(
        cls,
        msg: NodeStructure,
        ros_node: ROSNode,
        debug_manager: Optional[DebugManager] = None,
        subtree_manager: Optional[SubtreeManager] = None,
        logging_manager: Optional[LoggingManager] = None,
        data_flow_manager: Optional[DataFlowManager] = None,
    ) -> Result["Node", BehaviorTreeException]:
        """
        Construct a Node from the given ROS message.

        This will try to import the requested node class, instantiate it
        and populate its `name`, `options`, `input` and `output` members
        from the ROS message.

        This also catches exceptions raised during node construction
        and returns wrapped in as `result.Err`.

        :param ros_bt_py_msgs.msg.Node msg:

        A ROS message describing a node class. The node class must be
        available in the current environment (but does not need to be
        imported before calling this).

        :param debug_manager:

        The debug manager to use for the newly instantiated node class.

        :returns:

        An instance of the class named by `msg`, populated with the
        values from `msg`.

        Note that this does *not* include the node's state. Any node
        created by this will be in state UNININITIALIZED.

        :return:

        BehaviorTreeException if
        node cannot be instantiated.
        """
        if (
            msg.module not in cls.node_classes
            or msg.node_class not in cls.node_classes[msg.module]
        ):
            # If the node class was not available, try to load it
            load_node_module(msg.module)

        # If loading didn't work, abort
        if (
            msg.module not in cls.node_classes
            or msg.node_class not in cls.node_classes[msg.module]
        ):
            return Err(
                BehaviorTreeException(
                    "Failed to instantiate node from message - node class not available. "
                )
            )

        node_class = Node.node_classes[msg.module][msg.node_class]

        new_inputs: dict[str, DataContainer] = {}
        reference_values: dict[str, str] = {}
        io_msg: NodeIO
        for io_msg in msg.inputs:
            match get_iotype_for_msg(io_msg.type):
                case Err(e):
                    return Err(NodeConfigError(e))
                case Ok(c):
                    container = c
            new_inputs[io_msg.key] = container
            if isinstance(container, ReferenceContainer):
                reference_values[io_msg.key] = io_msg.serialized_value
            elif container.is_static:
                container.deserialize_value(io_msg.serialized_value)
        for key, value in reference_values.items():
            container = new_inputs[key]
            if not isinstance(container, ReferenceContainer):
                # This should NEVER happen given how `reference_values` was built
                continue
            container.set_type_map(new_inputs)
            if container.is_static:
                container.deserialize_value(value)

        match ros_to_uuid(msg.node_id):
            case Err(e):
                return Err(BehaviorTreeException(e))
            case Ok(n_id):
                node_id = n_id

        # Instantiate node - this shouldn't do anything yet, since we don't
        # call setup()
        try:
            node_instance = node_class(
                node_id=node_id,
                name=msg.name if msg.name else None,
                new_inputs=new_inputs,
                ros_node=ros_node,
                debug_manager=debug_manager,
                subtree_manager=subtree_manager,
                logging_manager=logging_manager,
                data_flow_manager=data_flow_manager,
            )
        except BehaviorTreeException as ex:
            return Err(ex)

        return Ok(node_instance)

    @typechecked
    def get_children_recursive(self) -> Iterator["Node"]:
        """Return all nodes that are below this node in the parent-child hirachy recursively."""
        yield self
        for child in self.children:
            for child_rec in child.get_children_recursive():
                yield child_rec

    @typechecked
    def get_subtree_msg(
        self,
    ) -> Result[
        Tuple[TreeStructure, list[Wiring], list[Wiring]], BehaviorTreeException
    ]:
        """
        Populate a TreeMsg with the subtree rooted at this node.

        This can be used to "shove" a subtree to a different host, by
        using that host's load_tree service.

        The subtree message will have public node data for every piece
        of node data that is wired to a node outside the subtree.

        :returns:

        A tuple consisting of a :class:`ros_bt_py_msgs.msg.Tree`
        message and two lists of
        :class:`ros_bt_py_msgs.msg.NodeDataWiring` messages
        (incoming_connections and outgoing_connections). The latter
        can be used to determine what parameters need to be forwarded
        to / from the remote executor if the subtree is to be executed
        remotely.

        Crucially, the resulting subtree will not be tick-able until all
        the incoming wirings from external_connections have been
        connected.

        However, if the subtree is to be shoved to a different
        executor, it's enough for the incoming wirings to be connected
        in the host tree - this will cause input values to be set and
        sent to the remote executor.

        """
        subtree_name = f"{self.name}_subtree"
        subtree = TreeStructure(
            name=subtree_name,
            root_id=uuid_to_ros(self.node_id),
            nodes=[node.to_structure_msg() for node in self.get_children_recursive()],
        )
        # This reassignment makes the typing happy,
        #   because they ensure that `.append` exists
        subtree.data_wirings = []

        node_map: dict[uuid.UUID, NodeStructure] = {
            # Since this is internal data, we assume ids to be safe
            ros_to_uuid(node.node_id).unwrap(): node
            for node in subtree.nodes
        }
        incoming_connections: list[Wiring] = []
        outgoing_connections: list[Wiring] = []
        for node in self.get_children_recursive():
            for sub in node.subscriptions:
                # Since this is internal data, we assume ids to be safe
                source_node = node_map.get(ros_to_uuid(sub.source.node_id).unwrap())
                target_node = node_map.get(ros_to_uuid(sub.target.node_id).unwrap())

                # For subscriptions where source and target are in the subtree,
                # add a wiring.
                if source_node and target_node:
                    subtree.data_wirings.append(
                        Wiring(source=sub.source, target=sub.target)
                    )
                # In the other cases, add that datum to public_node_data
                elif source_node:
                    outgoing_connections.append(sub)
                elif target_node:
                    incoming_connections.append(sub)
                else:
                    return Err(
                        BehaviorTreeException(
                            "Subscription in subtree has source *AND* target "
                            "outside of subtree!"
                        )
                    )

            for wiring, _, _ in node.subscribers:
                # Since this is internal data, we assume ids to be safe
                if ros_to_uuid(wiring.target.node_id).unwrap() not in node_map:
                    outgoing_connections.append(wiring)

        subtree.public_inputs = [
            connection.target for connection in incoming_connections
        ]
        subtree.public_outputs = [
            connection.source for connection in outgoing_connections
        ]

        return Ok((subtree, incoming_connections, outgoing_connections))

    @typechecked
    def to_structure_msg(self) -> NodeStructure:
        """
        Populate a ROS message with the information from this Node.

        Round-tripping the result through :meth:`Node.from_msg` should
        yield a working node object, with the caveat that state will not
        be preserved.

        :rtype: ros_bt_py_msgs.msg.Node
        :returns:

        A ROS message that describes the node.
        """
        node_type = type(self)

        return NodeStructure(
            module=node_type.__module__,
            node_class=node_type.__name__,
            name=self.name,
            node_id=uuid_to_ros(self.node_id),
            child_ids=[uuid_to_ros(child.node_id) for child in self.children],
            inputs=[
                NodeIO(
                    key=key,
                    type=container.serialize_type(),
                    # Only add serialized values for static inputs
                    serialized_value=(
                        container.serialize_value() if container.is_static else ""
                    ),
                )
                for key, container in self.node_config.inputs.items()
            ],
            outputs=[
                NodeIO(
                    key=key,
                    type=container.serialize_type(),
                )
                for key, container in self.node_config.outputs.items()
            ],
            max_children=(
                self.node_config.max_children
                if self.node_config.max_children is not None
                else -1
            ),
            tree_ref=(uuid_to_ros(self.tree_ref) if self.tree_ref is not None else ""),
        )

    def to_state_msg(self):
        return NodeState(node_id=uuid_to_ros(self.node_id), state=self.state)


N = TypeVar("N", bound=Node)


@typechecked
def define_bt_node(node_config: NodeConfig) -> Callable[[type[N]], type[N]]:
    """
    Provide information about this Node's interface.

    Every class that derives, directly or indirectly, from :class:`Node`,
    must be decorated with this!

    :param NodeConfig node_config:

    This describes your Node's interface. All inputs, outputs and
    options defined here are automatically registered with your
    class. You should not need to register anything manually!
    """

    def inner_dec(node_class: type[N]) -> type[N]:
        # Merge supplied node config with those of base classes
        for base in node_class.__bases__:
            if not hasattr(base, "_node_config"):
                continue
            match node_config.extend(base._node_config):
                case Err(e):
                    rclpy.logging.get_logger(node_class.__name__).error(
                        f"Node config could not be extended: {e}"
                    )
                    return node_class
                case Ok(None):
                    pass
        node_class._node_config = node_config

        if inspect.isabstract(node_class):
            # Don't register abstract classes
            rclpy.logging.get_logger(node_class.__name__).warning(
                f"Assigned NodeConfig to class {node_class.__name__}, but did not register "
                f"the class because it does not implement all required methods. "
                f"Missing methods: {', '.join(node_class.__abstractmethods__)}",
            )
            return node_class

        if node_class.__module__ not in Node.node_classes:
            Node.node_classes[node_class.__module__] = {node_class.__name__: node_class}
            return node_class

        if node_class.__name__ not in Node.node_classes[node_class.__module__]:
            Node.node_classes[node_class.__module__][node_class.__name__] = node_class
            return node_class

        rclpy.logging.get_logger(node_class.__name__).error(
            f"Node {node_class.__module__}.{node_class.__name__} was already registered,"
            "we don't support multiple configs for the same class."
        )
        return node_class

    return inner_dec


@typechecked
def load_node_module(package_name: str) -> Optional[ModuleType]:
    """
    Import the named module at run-time.

    If the module contains any (properly decorated) node classes,
    they will be registered and available to load via the other
    commands in this class.
    """
    try:
        return importlib.import_module(package_name)
    except (ImportError, ValueError) as exc:
        rclpy.logging.get_logger(package_name).error(
            f'Could not load node module "{package_name}": {repr(exc)}',
            throttle_duration_sec=30,
        )
        return None


@typechecked
def increment_name(name: str) -> str:
    """
    If `name` does not already end in a number, add "_2" to it.

    Otherwise, increase the number after the underscore.
    """
    match = re.search("_([0-9]+)$", name)
    prev_number = 1
    if match:
        prev_number = int(match.group(1))
        # remove the entire _$number part from the name
        name = name[: len(name) - len(match.group(0))]

    name += f"_{prev_number + 1}"
    return name


@define_bt_node(NodeConfig(inputs={}, outputs={}, max_children=1))
class Decorator(Node):
    """
    Base class for Decorator nodes.

    Decorators have exactly one child and somehow modify that child's
    output. Subclasses can add inputs and outputs, but never
    change `max_children`.
    """

    def _do_calculate_utility(self) -> Result[UtilityBounds, BehaviorTreeException]:
        """Pass on the utility value of the (only allowed) child."""
        if self.children:
            return self.children[0].calculate_utility()
        return Ok(
            UtilityBounds(
                can_execute=True,
                has_lower_bound_success=True,
                has_upper_bound_success=True,
                has_lower_bound_failure=True,
                has_upper_bound_failure=True,
            )
        )


@define_bt_node(NodeConfig(inputs={}, outputs={}, max_children=0))
class Leaf(Node):
    """
    Base class for leaf nodes in the tree.

    Leaf nodes have no children. Subclasses can define inputs, and outputs,
    but never change `max_children`.
    """


@define_bt_node(NodeConfig(inputs={}, outputs={}, max_children=None))
class FlowControl(Node):
    """
    Base class for flow control nodes.

    Flow control nodes (mostly Sequence, Fallback and their derivatives)
    can have an unlimited number of children and each have a unique set
    of rules for when to tick which of their children.
    """
