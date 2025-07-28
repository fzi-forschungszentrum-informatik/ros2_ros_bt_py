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
"""Module defining the Node class and helper functions representing a node in the behavior tree."""

from contextlib import contextmanager
from copy import deepcopy
from types import ModuleType

from result import Err, Ok, Result, as_result, is_err

import abc
import importlib
import inspect
import re
from typing import (
    Any,
    Callable,
    Generator,
    Iterable,
    List,
    Tuple,
    Type,
    Sequence,
    Dict,
    Optional,
)

import rclpy
import rclpy.logging
from rclpy.node import Node as ROSNode

from ros_bt_py_interfaces.msg import UtilityBounds
from typeguard import typechecked
from ros_bt_py_interfaces.msg import (
    NodeStructure,
    NodeState,
    NodeOption,
    NodeIO,
    NodeDataLocation,
    Wiring,
    WiringData,
    TreeStructure,
    TreeState,
    TreeData,
)

from ros_bt_py.debug_manager import DebugManager
from ros_bt_py.subtree_manager import SubtreeManager
from ros_bt_py.exceptions import (
    BehaviorTreeException,
    NodeStateError,
    NodeConfigError,
    TreeTopologyError,
)
from ros_bt_py.node_data import NodeData, NodeDataMap
from ros_bt_py.node_config import NodeConfig, OptionRef
from ros_bt_py.custom_types import TypeWrapper
from ros_bt_py.helpers import BTNodeState, get_default_value, json_decode, json_encode


@typechecked
def _check_node_data_match(
    node_config: Dict[str, Type], node_data: Iterable[NodeIO | NodeOption]
) -> bool:
    for data in node_data:
        try:
            value = node_config.get(data.key)
        except KeyError:
            return False
        if value is not data.serialized_type:
            return False
    return True


@typechecked
def _connect_wirings(data_wirings: List[Wiring], type: str) -> Dict[str, List[str]]:
    connected_wirings: Dict[str, List[str]] = {}
    for wiring in data_wirings:
        if wiring.source.data_kind == type:
            if wiring.source.node_name in connected_wirings:
                connected_wirings[wiring.source.node_name].append(
                    wiring.source.data_key
                )
            else:
                connected_wirings[wiring.source.node_name] = [wiring.source.data_key]
        elif wiring.target.data_kind == type:
            if wiring.target.node_name in connected_wirings:
                connected_wirings[wiring.target.node_name].append(
                    wiring.target.data_key
                )
            else:
                connected_wirings[wiring.target.node_name] = [wiring.target.data_key]
    return connected_wirings


@typechecked
def define_bt_node(node_config: NodeConfig) -> Callable[[Type["Node"]], Type["Node"]]:
    """
    Provide information about this Node's interface.

    Every class that derives, directly or indirectly, from :class:`Node`,
    must be decorated with this!

    :param NodeConfig node_config:

    This describes your Node's interface. All inputs, outputs and
    options defined here are automatically registered with your
    class. You should not need to register anything manually!
    """

    def inner_dec(node_class: Type[Node]) -> Type[Node]:
        # Merge supplied node config with those of base classes
        for base in node_class.__bases__:
            if hasattr(base, "_node_config") and base._node_config:
                config_extend_result = node_config.extend(base._node_config)
                if config_extend_result.is_err():
                    rclpy.logging.get_logger(node_class.__name__).error(
                        f"Node config could not be extended: {config_extend_result.unwrap_err()}"
                    )
        node_class._node_config = node_config

        if inspect.isabstract(node_class):
            # Don't register abstract classes
            rclpy.logging.get_logger(node_class.__name__).warn(
                f"Assigned NodeData to class {node_class.__name__}, but did not register "
                f"the class because it does not implement all required methods. "
                f"Missing methods: {', '.join(node_class.__abstractmethods__)}",
            )
            return node_class

        if node_class.__module__ not in Node.node_classes:
            Node.node_classes[node_class.__module__] = {
                node_class.__name__: [node_class]
            }
        else:
            if node_class.__name__ not in Node.node_classes[node_class.__module__]:
                Node.node_classes[node_class.__module__][node_class.__name__] = [
                    node_class
                ]
            else:

                def __check_dict_equiv(
                    dict1: Dict[str, Type], dict2: Dict[str, Type]
                ) -> bool:
                    if not len(dict1) == len(dict2):
                        return False

                    for key, value in dict1.items():
                        if key not in dict2:
                            return False
                        if not dict2[key] == value:
                            return False
                    return True

                already_available_node_classes = Node.node_classes[
                    node_class.__module__
                ][node_class.__name__]
                candidates = list(
                    filter(
                        lambda node_class_candidate: node_class_candidate._node_config
                        is not None
                        and __check_dict_equiv(
                            node_class_candidate._node_config.inputs, node_config.inputs
                        )
                        and __check_dict_equiv(
                            node_class_candidate._node_config.outputs,
                            node_config.outputs,
                        )
                        and __check_dict_equiv(
                            node_class_candidate._node_config.options,
                            node_config.options,
                        ),
                        already_available_node_classes,
                    )
                )
                if len(candidates) < 1:
                    Node.node_classes[node_class.__module__][
                        node_class.__name__
                    ].append(node_class)
                else:
                    rclpy.logging.get_logger(node_class.__name__).error(
                        "Node class is already registered with this config!"
                    )
        return node_class

    return inner_dec


class NodeMeta(abc.ABCMeta):
    """
    Override the __doc__ property to add a list of BT params.

    (inputs, outputs and options) to every node class.
    """

    _node_config: Optional[NodeConfig] = None
    _doc: str = ""

    def __new__(cls, name, bases, attrs):
        """Add doc attribute to the new NodeMeta class."""
        attrs["_doc"] = attrs.get("__doc__", "")
        return super(NodeMeta, cls).__new__(cls, name, bases, attrs)

    @property
    def __doc__(self) -> Optional[str]:
        """Generate documentation depending on the node configuration."""
        if (
            hasattr(self, "_node_config")
            and self._node_config is not None
            and (
                self._node_config.inputs
                or self._node_config.outputs
                or self._node_config.options
            )
        ):
            # Build table of inputs, outputs and options
            # Start with two newlines to separate from the original docstring
            param_table = ["\n\n" "**Behavior Tree I/O keys**\n\n"]
            if self._node_config.options:
                param_table.append("*Options*\n\n")
                for option_key in self._node_config.options:
                    if isinstance(self._node_config.options[option_key], OptionRef):
                        param_table.append(
                            f"* {option_key}: ``{str(self._node_config.options[option_key])}``\n"
                        )
                    elif isinstance(self._node_config.options[option_key], TypeWrapper):
                        param_table.append(
                            f"* {option_key}: :class:"
                            f"`{self._node_config.options[option_key].actual_type.__name__}` "
                            f"(`{self._node_config.options[option_key].info}`)\n"
                        )
                    else:
                        param_table.append(
                            f"* {option_key}: :class:"
                            f"`{self._node_config.options[option_key].__name__}`\n"
                        )
                param_table.append("\n")
            if self._node_config.inputs:
                param_table.append("*Inputs*\n\n")
                for input_key in self._node_config.inputs:
                    if isinstance(self._node_config.inputs[input_key], OptionRef):
                        param_table.append(
                            f"* {input_key}: ``{str(self._node_config.inputs[input_key])}``\n"
                        )
                    else:
                        param_table.append(
                            f"* {input_key}: :class:"
                            f"`{self._node_config.inputs[input_key].__name__}`\n"
                        )
                param_table.append("\n")
            if self._node_config.outputs:
                param_table.append("*Outputs*\n\n")
                for output_key in self._node_config.outputs:
                    if isinstance(self._node_config.outputs[output_key], OptionRef):
                        param_table.append(
                            f"* {output_key}: ``{str(self._node_config.outputs[output_key])}``\n"
                        )
                    else:
                        param_table.append(
                            f"* {output_key}: :class:"
                            f"`{self._node_config.outputs[output_key].__name__}`\n"
                        )
                param_table.append("\n")

            return self._doc + "".join(param_table)
        else:
            return self._doc


class Node(object, metaclass=NodeMeta):
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

    @contextmanager
    def _dummy_report_state(self):
        self.logdebug("Reporting state up without debug manager")
        yield

    @contextmanager
    def _dummy_report_tick(self):
        self.logdebug("Ticking without debug manager")
        yield

    node_classes: Dict[str, Dict[str, List[Type["Node"]]]] = {}
    _node_config: Optional[NodeConfig] = None
    permissive: bool = False
    debug_manager: Optional[DebugManager]
    subtree_manager: Optional[SubtreeManager]
    _state: BTNodeState

    def __init__(
        self,
        options: Optional[Dict] = None,
        debug_manager: Optional[DebugManager] = None,
        subtree_manager: Optional[SubtreeManager] = None,
        name: Optional[str] = None,
        ros_node: Optional[ROSNode] = None,
    ) -> None:
        """
        Prepare class members.

        After this finishes, the Node is *not* ready to run. You still
        need to do your own initialization in :meth:`_do_setup`.

        Since the `__init__` method has to return None,
        it doesn't use `result` but raises errors as normal.
        The `from_msg` method catches any errors and passes them on as a `result.Err`.

        :param dict options: Map from option names to option values. Use these for configuring
        your node, do not provide a custom `__init()__` method!

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
        # Only used to make finding the root of the tree easier
        self.parent: Optional[Node] = None
        self._state: BTNodeState = BTNodeState.UNINITIALIZED
        self.children: List[Node] = []

        self.subscriptions: list[Wiring] = []
        self.subscribers: list[tuple[Wiring, Callable[[type], None], type]] = []

        self._ros_node: Optional[ROSNode] = ros_node
        self.debug_manager: Optional[DebugManager] = debug_manager
        self.subtree_manager: Optional[SubtreeManager] = subtree_manager

        if not self._node_config:
            raise NodeConfigError("Missing node_config, cannot initialize!")

        # Copy the class NodeConfig so we can mess with it (but we
        # only should in very rare cases!)
        self.node_config = deepcopy(self._node_config)

        self.options = NodeDataMap(name="options")
        register_result = self._register_node_data(
            source_map=self.node_config.options,
            target_map=self.options,
            values=options,
            permissive=self.permissive,
        )
        if register_result.is_err():
            raise register_result.unwrap_err()

        # Warn about unset options, ignore missing optional_options
        unset_option_keys = [
            key for key in self.options if options is None or key not in options
        ]
        if unset_option_keys:
            optional_keys = []
            for key in unset_option_keys:
                if key in self.node_config.optional_options:
                    optional_keys.append(key)
            if unset_option_keys == optional_keys:
                rclpy.logging.get_logger(self.name).warn(
                    f"missing optional keys: {optional_keys}"
                )
            else:
                raise ValueError(f"Missing options: {str(unset_option_keys)}")

        # Warn about extra options
        if options is not None:
            extra_option_keys = [key for key in options if key not in self.options]
            if extra_option_keys:
                raise ValueError(f"Extra options: {str(extra_option_keys)}")

        self.inputs = NodeDataMap(name="inputs")
        register_result = self._register_node_data(
            source_map=self.node_config.inputs, target_map=self.inputs
        )
        if register_result.is_err():
            raise register_result.unwrap_err()

        self.outputs = NodeDataMap(name="outputs")
        register_result = self._register_node_data(
            source_map=self.node_config.outputs, target_map=self.outputs
        )
        if register_result.is_err():
            raise register_result.unwrap_err()

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
            error_msg = f"{self.name} nodes not have ROS node reference!"
            self.logerr(error_msg)
            raise RuntimeError(error_msg)

    @ros_node.setter
    @typechecked
    def ros_node(self, new_ros_node: ROSNode):
        self.logdebug(f"Setting new ROS node: {new_ros_node}")
        self._ros_node = new_ros_node

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
            setup_result = self._do_setup()
            self._setup_called = True

            if setup_result.is_err():
                self.state = BTNodeState.BROKEN
            else:
                self.state = setup_result.unwrap()

        return setup_result

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

    def _handle_inputs(self) -> Result[None, str]:
        """
        Execute the callbacks registered by :meth:`_wire_input`.

        But only if an input has been updated since the last tick.
        """
        for input_name in self.inputs:
            if not self.inputs.is_updated(input_name):
                continue
            if self.inputs[input_name] is None:
                # Omit the Error if we declared it to be "okay"
                # This might still not be the best solution but enables some flexibility
                if input_name not in self.node_config.optional_options:
                    return Err(
                        f"Trying to tick a node ({self.name}) with an unset input ({input_name})!"
                    )
        return Ok(self.inputs.handle_subscriptions())

    def _handle_outputs(self) -> None:
        """
        Execute the callbacks registered by :meth:`NodeDataMap.subscribe`: .

        But only if the output has changed during this tick (see where
        the :meth:`NodeDataMap.reset_updated` is called in
        :meth:`tick`)
        """
        self.outputs.handle_subscriptions()

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

            unset_options: List[str] = []
            for option_name in self.options:
                if (
                    not self.options.is_updated(option_name)
                    and option_name not in self.node_config.optional_options
                ):
                    unset_options.append(option_name)
            if unset_options:
                msg = f"Trying to tick node with unset options: {str(unset_options)}"
                self.logwarn(msg)
                self.state = BTNodeState.BROKEN
                return Err(BehaviorTreeException(msg))
            self.options.handle_subscriptions()

            # Outputs are updated in the tick. To catch that, we need to reset here.
            self.outputs.reset_updated()

            # Inputs can override options!
            handle_input_result = self._handle_inputs()
            if handle_input_result.is_err():
                self.state = BTNodeState.BROKEN
                return Err(BehaviorTreeException(handle_input_result.err()))
            tick_result = self._do_tick()
            if tick_result.is_ok():
                self.state = tick_result.unwrap()
            else:
                self.state = BTNodeState.BROKEN
                return tick_result

            # Inputs are updated by other nodes' outputs, i.e. some time after
            # we use them here. In some cases, inputs might be connected to
            # child outputs (or even our own). If they are, update information
            # is lost, unless it is processed after all child ticks in the same
            # cycle!
            self.inputs.reset_updated()

            valid_state_result = self.check_if_in_invalid_state(
                allowed_states=[
                    BTNodeState.RUNNING,
                    BTNodeState.SUCCEEDED,
                    BTNodeState.FAILED,
                    BTNodeState.ASSIGNED,
                    BTNodeState.UNASSIGNED,
                ],
                action_name="tick()",
            )
            if valid_state_result.is_err():
                return Err(valid_state_result.unwrap_err())

            self._handle_outputs()

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
            untick_result = self._do_untick()
            if untick_result.is_ok():
                self.state = untick_result.unwrap()
            else:
                self.state = BTNodeState.BROKEN
                return untick_result

            check_state_result = self.check_if_in_invalid_state(
                allowed_states=[BTNodeState.IDLE, BTNodeState.PAUSED],
                action_name="untick()",
            )
            if check_state_result.is_err():
                return Err(check_state_result.unwrap_err())

            self.outputs.reset_updated()
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

            # Reset input/output reset state and set outputs to None
            # before calling _do_reset() - the node can overwrite the None
            # with more appropriate values if need be.
            self.inputs.reset_updated()

            for output_key in self.outputs:
                self.outputs[output_key] = None
            self.outputs.reset_updated()

            reset_result = self._do_reset()
            if reset_result.is_ok():
                self.state = reset_result.unwrap()
            else:
                self.state = BTNodeState.BROKEN
                return reset_result
            valid_state_result = self.check_if_in_invalid_state(
                allowed_states=[BTNodeState.IDLE], action_name="reset()"
            )

            if valid_state_result.is_err():
                return Err(valid_state_result.unwrap_err())

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
            error_result = None
            if self.state == BTNodeState.SHUTDOWN:
                self.loginfo(
                    "Not calling shutdown method, node has not been initialized yet"
                )
                self.state = BTNodeState.SHUTDOWN
                # Call shutdown on all children - this should only set
                # their state to shutdown
                for child in self.children:
                    shutdown_result = child.shutdown()
                    if shutdown_result.is_err():
                        self.logwarn(
                            f"Node {child.name} raised the following error during shutdown"
                            "Continuing to shutdown other nodes"
                            f"{shutdown_result.unwrap_err()}"
                        )
                        error_result = shutdown_result

            shutdown_result = self._do_shutdown()
            if shutdown_result.is_ok():
                self.state = shutdown_result.unwrap()
            else:
                self.state = BTNodeState.BROKEN
                error_result = shutdown_result

            for child in self.children:
                shutdown_result = child.shutdown()
                if shutdown_result.is_err():
                    self.logwarn(
                        f"Node {child.name} raised the following error during shutdown"
                        "Continuing to shutdown other nodes"
                        f"{shutdown_result.unwrap_err()}"
                    )
                    error_result = shutdown_result

            unshutdown_children = [
                f"{child.name} ({type(child).__name__}), state: {child.state}"
                for child in self.children
                if child.state != BTNodeState.SHUTDOWN
            ]
            if len(unshutdown_children) > 0:
                self.logwarn(
                    "Not all children are shut down after calling shutdown(). "
                    "List of not-shutdown children and states:\n"
                    f"{unshutdown_children}"
                )

            if error_result is not None:
                return error_result

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
    def get_child_index(self, child_name: str) -> Optional[int]:
        """
        Get the index in the `children` array of the child with the given name.

        This is useful if you want to replace a child with another node.

        :returns:

        An integer index if a child with the given name exists, `None`
        if there's no such child

        """
        try:
            return [child.name for child in self.children].index(child_name)
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

        if child.name in (child1.name for child1 in self.children):
            return Err(
                TreeTopologyError(f"Already have a child with name '{child.name}'")
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
    def remove_child(self, child_name: str) -> Result["Node", KeyError]:
        """
        Remove the child with the given name and return it.

        :param basestring child_name: The name of the child to remove
        """
        child_index = self.get_child_index(child_name)
        if child_index is None:
            return Err(KeyError(f'Node {self.name} has no child named "{child_name}"'))

        tmp = self.children[child_index]
        del self.children[child_index]
        tmp.parent = None
        return Ok(tmp)

    @staticmethod
    @typechecked
    def _find_option_refs(
        source_map: Dict[str, Any],
        target_map: NodeDataMap,
        values: Optional[Dict[str, Any]] = None,
        permissive: bool = False,
    ) -> Result[None, NodeConfigError]:
        for key, data_type in {
            k: v for (k, v) in source_map.items() if not isinstance(v, OptionRef)
        }.items():
            if key in target_map:
                return Err(NodeConfigError(f"Duplicate data name: {key}"))
            add_result = target_map.add(key, NodeData(data_type=data_type))
            if add_result.is_err():
                return Err(NodeConfigError(add_result.unwrap_err()))
            if values is not None and key in values:
                try:
                    target_map[key] = values[key]
                except TypeError:
                    if permissive:
                        if data_type == type:
                            target_map[key] = int
                            # if data_type is a type check if a OptionRef exists
                            for key_opt, value_opt in source_map.items():
                                if isinstance(value_opt, OptionRef):
                                    if value_opt.option_key == key:
                                        values[key_opt] = 0
                                        if key_opt in target_map:
                                            # overwrite already set target_map
                                            target_map[key_opt] = 0
                        else:
                            return Err(
                                NodeConfigError(
                                    f"Value does not match destination type for: {key} "
                                    "and is not a type!"
                                )
                            )
                    else:
                        return Err(
                            NodeConfigError(
                                f"Value does not match destination type for: {key} "
                                "and not in permissive mode!"
                            )
                        )
        return Ok(None)

    @typechecked
    def _register_node_data(
        self,
        source_map: Dict[str, Any],
        target_map: NodeDataMap,
        values: Optional[Dict[str, Any]] = None,
        permissive: bool = False,
    ) -> Result[None, NodeConfigError]:
        """
        Register a number of typed :class:`NodeData` in the given map.

        Note that when using :class:`OptionRef`, the option keys
        referenced by any :class:`OptionRef` objects must exist and be
        populated!

        :param dict(str, type) source_map: a dictionary mapping from data keys to data types,
        i.e. ``{ 'a_string' : str, 'an_int' : int }``

        :param NodeDataMap target_map:
        The :class:`NodeDataMap` to add :class:`NodeData` values to

        :param dict(str, value) values:
        An optional dictionary containing the values for the NodeData

        :returns: NodeConfigError in any of the following cases:
          * If any of the keys in `source_map` already exist in `target_map`
          * If an OptionRef value is passed, but `allow_ref` is `False`
          * If an OptionRef references an option value that has not been set or
            does not exist
          * If an OptionRef references an option value that does not hold a `type`

        """
        # Find the values that are not OptionRefs first
        find_option_refs_result = self._find_option_refs(
            source_map=source_map,
            target_map=target_map,
            permissive=permissive,
            values=values,
        )
        if find_option_refs_result.is_err():
            return find_option_refs_result

        # Now process OptionRefs
        for key, data_type in {
            k: v for (k, v) in source_map.items() if isinstance(v, OptionRef)
        }.items():
            if key in target_map:
                return Err(
                    NodeConfigError(f"Duplicate {target_map.name} data name: {key}")
                )

            if data_type.option_key not in self.options:
                return Err(
                    NodeConfigError(
                        f'OptionRef for {target_map.name} key "{key}" references invalid '
                        f'option key "{data_type.option_key}"'
                    )
                )
            if not self.options.is_updated(data_type.option_key):
                return Err(
                    NodeConfigError(
                        f'OptionRef for {target_map.name} key "{key}" references unwritten '
                        f'option key "{data_type.option_key}"'
                    )
                )
            if not isinstance(self.options[data_type.option_key], type):
                return Err(
                    NodeConfigError(
                        f'OptionRef for {target_map.name} key "{key}" references option key '
                        f'"{data_type.option_key}" that does not contain a type!'
                    )
                )
            add_result = target_map.add(
                key, NodeData(data_type=self.options[data_type.option_key])
            )
            if add_result.is_err():
                return Err(NodeConfigError(add_result.unwrap_err()))
            if values is not None and key in values:
                try:
                    target_map[key] = values[key]
                except AttributeError as e:
                    if permissive:
                        if (
                            type(values[key]).__slots__ is not None
                            and type(values[key])._slot_types is not None
                        ):
                            fixed_new_value = type(values[key])()

                            for _, slot in enumerate(type(values[key]).__slots__):
                                setattr(
                                    fixed_new_value,
                                    slot,
                                    getattr(
                                        values[key],
                                        slot,
                                        get_default_value(
                                            type(getattr(fixed_new_value, slot, int)),
                                            ros=True,
                                        ),
                                    ),
                                )
                            target_map[key] = fixed_new_value
                        else:
                            return Err(
                                NodeConfigError(
                                    "Types do not match, "
                                    f"did ROS Message definitons change: {str(e)}"
                                )
                            )
                    else:
                        return Err(
                            NodeConfigError(
                                "Types do not match, "
                                f"did ROS Message definitons change: {str(e)}"
                            )
                        )
        return Ok(None)

    def __repr__(self) -> str:
        """Create a string representation of the node class."""
        return (
            f"{type(self).__name__}("
            f"options={({key: self.options[key] for key in self.options})}, "
            f"name={self.name}), "
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
            self.name == other.name
            and self.parent == other.parent
            and self.state == other.state
            and type(self).__module__ == type(other).__module__
            and type(self).__name__ == type(other).__name__
            and self.options == other.options
            and self.inputs == other.inputs
            and self.outputs == other.outputs
            and self.children == other.children
        )

    @typechecked
    def __ne__(self, other: Any) -> bool:
        """Check if two nodes have a single differing attribute."""
        return not self == other

    @typechecked
    def get_data_map(self, data_kind: str) -> Result[NodeDataMap, KeyError]:
        """
        Return one of our NodeDataMaps by string name.

        :param basestring data_kind:
          One of the constants in :class:`ros_bt_py_msgs.msg.NodeDataLocation`

        :rtype: NodeDataMap
        """
        if data_kind == NodeDataLocation.INPUT_DATA:
            return Ok(self.inputs)
        if data_kind == NodeDataLocation.OUTPUT_DATA:
            return Ok(self.outputs)
        if data_kind == NodeDataLocation.OPTION_DATA:
            return Ok(self.options)

        return Err(
            KeyError(
                f"{data_kind} is not a valid value to pass to Node.get_data_map()!"
            )
        )

    # Logging methods - these just use the ROS logging framework, but add the
    # name and type of the node so it's easier to trace errors.

    @typechecked
    def logdebug(self, message: str) -> None:
        """
        Wrap call to :func:rclpy.logger.get_logger(...).debug.

        Adds this node's name and type to the given message
        """
        rclpy.logging.get_logger(self.name).debug(f"{message}")

    @typechecked
    def loginfo(self, message: str) -> None:
        """
        Wrap call to :func:rclpy.logging.get_logger(...).info.

        Adds this node's name and type to the given message
        """
        rclpy.logging.get_logger(self.name).info(f"{message}")

    @typechecked
    def logwarn(self, message: str) -> None:
        """
        Wrap call to :func:rclpy.logging.get_logger(...).warn.

        Adds this node's name and type to the given message
        """
        rclpy.logging.get_logger(self.name).warn(f"{message}")

    @typechecked
    def logerr(self, message: str) -> None:
        """
        Wrap call to :func:rclpy.logging.get_logger(...).error.

        Adds this node's name and type to the given message
        """
        rclpy.logging.get_logger(self.name).error(f"{message}")

    @typechecked
    def logfatal(self, message: str) -> None:
        """
        Wrap call to :func:rclpy.logging.get_logger(...).fatal.

        Adds this node's name and type to the given message
        """
        rclpy.logging.get_logger(self.name).fatal(f"{message}")

    @classmethod
    @typechecked
    def from_msg(
        cls: Type["Node"],
        msg: NodeStructure,
        ros_node: ROSNode,
        debug_manager: Optional[DebugManager] = None,
        subtree_manager: Optional[SubtreeManager] = None,
        permissive: bool = False,
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

        :param permissive:

        Enable permissive behavior.

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
                    f"Original message:\n{str(msg)}"
                )
            )

        node_classes: List[Type[Node]] = cls.node_classes[msg.module][msg.node_class]

        node_class: Optional[Type[Node]] = None
        if len(node_classes) > 1:
            candidates: List[Type[Node]] = list(
                filter(
                    lambda node_class_candidate: node_class_candidate._node_config
                    is not None
                    and _check_node_data_match(
                        node_class_candidate._node_config.inputs, msg.inputs
                    )
                    and _check_node_data_match(
                        node_class_candidate._node_config.outputs, msg.outputs
                    )
                    and _check_node_data_match(
                        node_class_candidate._node_config.options, msg.options
                    ),
                    node_classes,
                )
            )
            if len(candidates) < 1:
                return Err(
                    BehaviorTreeException(
                        "Failed to instantiate node from message - node class not available."
                        f"Original message:\n{str(msg)}"
                    )
                )
            if len(candidates) > 1:
                return Err(
                    BehaviorTreeException(
                        "Failed to instantiate node from message - "
                        "multiple versions of node class "
                        f"available. Original message:\n {str(msg)}"
                    )
                )
            node_class = candidates[0]
        else:
            node_class = node_classes[0]

        # Populate options dict
        options_dict: Dict[str, Any] = {}
        try:
            for option in msg.options:
                options_dict[option.key] = json_decode(option.serialized_value)
        except ValueError as exc:
            return Err(
                BehaviorTreeException(
                    f"Failed to instantiate node from message: {str(exc)}"
                )
            )

        # Instantiate node - this shouldn't do anything yet, since we don't
        # call setup()
        node_class.permissive = permissive
        try:
            if msg.name:
                node_instance = node_class(
                    name=msg.name,
                    options=options_dict,
                    debug_manager=debug_manager,
                    subtree_manager=subtree_manager,
                    ros_node=ros_node,
                )
            else:
                node_instance = node_class(
                    options=options_dict,
                    debug_manager=debug_manager,
                    subtree_manager=subtree_manager,
                    ros_node=ros_node,
                )
        except BehaviorTreeException as ex:
            return Err(ex)

        return Ok(node_instance)

    @typechecked
    def get_children_recursive(self) -> Generator["Node", None, None]:
        """Return all nodes that are below this node in the parent-child hirachy recursively."""
        yield self
        for child in self.children:
            for child_rec in child.get_children_recursive():
                yield child_rec

    @typechecked
    def get_subtree_msg(
        self,
    ) -> Result[
        Tuple[TreeStructure, List[Wiring], List[Wiring]], BehaviorTreeException
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
            root_name=self.name,
            nodes=[node.to_structure_msg() for node in self.get_children_recursive()],
        )
        # These reassignments makes the typing happy,
        #   because they ensure that `.append` exists
        subtree.data_wirings = []
        subtree.public_node_data = []

        node_map: Dict[str, NodeStructure] = {node.name: node for node in subtree.nodes}
        incoming_connections: List[Wiring] = []
        outgoing_connections: List[Wiring] = []
        for node in self.get_children_recursive():
            for sub in node.subscriptions:
                source_node = node_map.get(sub.source.node_name)
                target_node = node_map.get(sub.target.node_name)

                # For subscriptions where source and target are in the subtree,
                # add a wiring.
                if source_node and target_node:
                    subtree.data_wirings.append(
                        Wiring(source=sub.source, target=sub.target)
                    )
                # In the other cases, add that datum to public_node_data
                elif source_node:
                    subtree.public_node_data.append(sub.source)
                    outgoing_connections.append(sub)
                elif target_node:
                    subtree.public_node_data.append(sub.target)
                    incoming_connections.append(sub)
                else:
                    return Err(
                        BehaviorTreeException(
                            "Subscription in subtree has source *AND* target "
                            "outside of subtree!"
                        )
                    )

            for wiring, _, _ in node.subscribers:
                if wiring.target.node_name not in node_map:
                    subtree.public_node_data.append(wiring.source)
                    outgoing_connections.append(wiring)

        connected_inputs = _connect_wirings(
            subtree.data_wirings, NodeDataLocation.INPUT_DATA
        )
        connected_outputs = _connect_wirings(
            subtree.data_wirings, NodeDataLocation.OUTPUT_DATA
        )

        for node in subtree.nodes:
            for node_input in node.inputs:
                if (
                    node.name not in connected_inputs
                    or node_input.key not in connected_inputs[node.name]
                ):
                    # Input is unconnected, list it as public
                    subtree.public_node_data.append(
                        NodeDataLocation(
                            node_name=node.name,
                            data_kind=NodeDataLocation.INPUT_DATA,
                            data_key=node_input.key,
                        )
                    )
            for node_output in node.outputs:
                if (
                    node.name not in connected_outputs
                    or node_output.key not in connected_outputs[node.name]
                ):
                    # Input is unconnected, list it as public
                    subtree.public_node_data.append(
                        NodeDataLocation(
                            node_name=node.name,
                            data_kind=NodeDataLocation.OUTPUT_DATA,
                            data_key=node_output.key,
                        )
                    )
        return Ok((subtree, incoming_connections, outgoing_connections))

    @typechecked
    def find_node(self, other_name: str) -> Optional["Node"]:
        """
        Try to find the node with the given name in the tree.

        This is not a particularly cheap operation, since it ascends
        the tree up to the root and then recursively descends back
        until it finds the node.

        Probably best not to use it in a tick function.

        """
        root = self
        while root.parent is not None:
            root = root.parent

        for node in root.get_children_recursive():
            if node.name == other_name:
                return node

        return None

    @typechecked
    def _subscribe(
        self,
        wiring: Wiring,
        new_cb: Callable[[Type], None],
        expected_type: Type,
    ) -> Result[None, BehaviorTreeException]:
        """
        Subscribe to a piece of Nodedata this node has.

        Call this on a node to *subscribe to NodeData **from** that
        node*!

        :param `ros_bt_py_msgs.msg.NodeDataWiring` wiring:

        Defines the source and target of the subscribe operation.
        `wiring.source` must point to a valid data key in this node
        (`self`).  `wiring.target` is not checked here (because it's
        only used to label subscriptions), but it should also point to a
        valid data key of a node in the tree.

        :param cb:

        A callback function that will be called whenever there's an updated
        value for the key (and this node receives a tick)

        :param `type` expected_type:

        The type that the subscriber expects our piece of data to
        have. If it doesn't match the type of the data at the requested
        key, raise a `BehaviorTreeException`.

        :returns:

        BehaviorTreeException if `expected_type` and the actual type of
        the data are incompatible.
        """
        if wiring.source.node_name != self.name:
            return Err(
                BehaviorTreeException(
                    f"{self.name}: Trying to subscribe to another node ({wiring.source.node_name})"
                )
            )

        for sub, _, _ in self.subscribers:
            if sub.target == wiring.target:
                if sub.source == wiring.source:
                    return Err(BehaviorTreeException("Duplicate subscription!"))
                self.logwarn(
                    f"Subscriber {wiring.target.node_name} is subscribing to multiple sources "
                    f"with the same target {wiring.target.data_kind}[{wiring.target.data_key}]"
                )

        source_map_result = self.get_data_map(wiring.source.data_kind)
        if source_map_result.is_err():
            return Err(BehaviorTreeException(str(source_map_result.unwrap_err())))

        source_map = source_map_result.unwrap()

        if wiring.source.data_key not in source_map:
            return Err(
                BehaviorTreeException(
                    f"Source key {self.name}.{wiring.source.data_kind}[{wiring.source.data_key}] "
                    "does not exist!"
                )
            )

        if not issubclass(source_map.get_type(wiring.source.data_key), expected_type):
            return Err(
                BehaviorTreeException(
                    f"Type of {self.name}.{wiring.source.data_kind}[{wiring.source.data_key}] "
                    f"({source_map.get_type(wiring.source.data_key).__name__}) "
                    "is not compatible with Type of "
                    f"{wiring.target.node_name}."
                    f"{wiring.target.data_kind}"
                    f"[{wiring.target.data_key}] "
                    f"({expected_type})!"
                )
            )

        source_map.subscribe(
            wiring.source.data_key,
            new_cb,
            f"{wiring.target.node_name}.{wiring.target.data_kind}[{wiring.target.data_key}]",
        )
        self.subscribers.append((deepcopy(wiring), new_cb, expected_type))
        return Ok(None)

    @typechecked
    def wire_data(self, wiring: Wiring) -> Result[None, BehaviorTreeException]:
        """
        Wire a piece of Nodedata from another node to this node.

        Call this on a node to *connect it to NodeData from
        **another** node*!

        :param `ros_bt_py_msgs.msg.NodeDataWiring` wiring:

        Indicates both the piece of NodeData we want to subscribe to
        (`wiring.source`) and the data key inside this node we want to
        connect it to (`wiring.target`).


        :param `ros_bt_py_msgs.msg.NodeDataLocation` target:
           The position *inside this node* we want to wire *to*.

        :returns:

        BehaviorTreeException if a subscription with the same source and
        target exists already, or if the types of source and target data
        are incompatible.
        """
        if wiring.target.node_name != self.name:
            return Err(
                BehaviorTreeException(
                    f"Target of wiring ({wiring.target.node_name}) is not this node ({self.name})"
                )
            )

        for sub in self.subscriptions:
            if sub.target == wiring.target:
                if sub.source == wiring.source:
                    return Err(BehaviorTreeException("Duplicate subscription!"))

        source_node = self.find_node(wiring.source.node_name)
        if not source_node:
            return Err(
                BehaviorTreeException(
                    f"Source node {wiring.source.node_name} does not exist or is not connected "
                    f"to target node {self.name}"
                )
            )

        source_map_result = source_node.get_data_map(wiring.source.data_kind)
        if source_map_result.is_err():
            return Err(BehaviorTreeException(str(source_map_result.unwrap_err())))

        source_map = source_map_result.unwrap()
        if wiring.source.data_key not in source_map:
            return Err(
                BehaviorTreeException(
                    f"Source key {source_node.name}."
                    f"{wiring.source.data_kind}[{wiring.source.data_key}] does not exist!"
                )
            )

        target_map_result = self.get_data_map(wiring.target.data_kind)
        if target_map_result.is_err():
            return Err(BehaviorTreeException(str(target_map_result.unwrap_err())))
        target_map = target_map_result.unwrap()

        if wiring.target.data_key not in target_map:
            return Err(
                BehaviorTreeException(
                    f"Target key {self.name}."
                    f"{wiring.target.data_kind}[{wiring.target.data_key}] does not exist!"
                )
            )

        subscribe_result = source_node._subscribe(
            wiring,
            target_map.get_callback(wiring.target.data_key),
            target_map.get_type(wiring.target.data_key),
        )
        if subscribe_result.is_err():
            return subscribe_result

        self.subscriptions.append(deepcopy(wiring))
        return Ok(None)

    @typechecked
    def _unsubscribe(self, wiring: Wiring) -> Result[None, BehaviorTreeException]:
        """
        Unsubscribe from a piece of NodeData this node has.

        Call this to undo a call to `Node._subscribe()'

        :return:

        BehaviorTreeException if the requested data location is not in this node, or
        the requested key does not exist in this node.

        """
        if wiring.source.node_name != self.name:
            return Err(
                BehaviorTreeException(
                    f"{self.name}: Trying to unsubscribe from another node "
                    f"({wiring.source.node_name})"
                )
            )
        source_map_result = self.get_data_map(wiring.source.data_kind)
        if source_map_result.is_err():
            return Err(BehaviorTreeException(str(source_map_result.unwrap_err())))
        source_map = source_map_result.unwrap()

        if wiring.source.data_key not in source_map:
            return Err(
                BehaviorTreeException(
                    f"Source key {self.name}."
                    f"{wiring.source.data_kind}[{wiring.source.data_key}] does not exist!"
                )
            )

        for sub_wiring, callback, _ in self.subscribers:
            if wiring.target == sub_wiring.target:
                source_map.unsubscribe(wiring.source.data_key, callback)
        # remove subscriber data from list
        self.subscribers = [
            sub for sub in self.subscribers if sub[0].target != wiring.target
        ]
        return Ok(None)

    @typechecked
    def unwire_data(self, wiring: Wiring) -> Result[None, BehaviorTreeException]:
        """
        Unwire the given wiring.

        This entails finding the source of the wiring, calling its
        :meth:`Node._unsubscribe` method and removing the wiring from
        this node's list of subscriptions.

        :return: BehaviorTreeException

        If the given wiring's source node cannot be found from this
        node.
        """
        if wiring.target.node_name != self.name:
            return Err(
                BehaviorTreeException(
                    f"Target of wiring ({wiring.target.node_name}) is not this node ({self.name})"
                )
            )
        source_node = self.find_node(wiring.source.node_name)

        if wiring not in self.subscriptions:
            # Nothing to do
            return Ok(None)

        if source_node:
            unsubscribe_result = source_node._unsubscribe(wiring)
            if unsubscribe_result.is_err():
                return unsubscribe_result

        self.subscriptions.remove(wiring)

        # If the removed wiring was the last subscription for this
        # datum, set it to None
        if not [
            sub
            for sub in self.subscriptions
            if (
                sub.target.data_kind == wiring.target.data_kind
                and sub.target.data_key == wiring.target.data_key
            )
        ]:
            if wiring.target.data_kind == NodeDataLocation.INPUT_DATA:
                self.inputs[wiring.target.data_key] = None
            elif wiring.target.data_kind == NodeDataLocation.OUTPUT_DATA:
                self.outputs[wiring.target.data_key] = None
            elif wiring.target.data_kind == NodeDataLocation.OPTION_DATA:
                self.options[wiring.target.data_key] = None
        return Ok(None)

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
            version=self.node_config.version,
            name=self.name,
            child_names=[child.name for child in self.children],
            options=[
                NodeOption(
                    key=key,
                    serialized_value=self.options.get_serialized(key),
                    serialized_type=self.options.get_serialized_type(key),
                )
                for key in self.options
            ],
            inputs=[
                NodeIO(
                    key=key,
                    serialized_type=self.inputs.get_serialized_type(key),
                )
                for key in self.inputs
            ],
            outputs=[
                NodeIO(
                    key=key,
                    serialized_type=self.outputs.get_serialized_type(key),
                )
                for key in self.outputs
            ],
            max_children=(
                self.node_config.max_children
                if self.node_config.max_children is not None
                else -1
            ),
        )

    def to_state_msg(self):
        return NodeState(name=self.name, state=self.state)

    def wire_data_msg_list(self):
        data_list: list[WiringData] = []
        for wiring, _, exp_type in self.subscribers:
            # Since we iterate subscribers, `wiring.source` should refer to self.
            source_map_result = self.get_data_map(wiring.source.data_kind)
            if source_map_result.is_err():
                continue
            source_map = source_map_result.unwrap()
            key = wiring.source.data_key
            if not source_map.is_updated(key):
                continue  # Don't publish stale data
            wiring_data = WiringData()
            wiring_data.wiring = wiring
            wiring_data.serialized_data = source_map.get_serialized(key)
            wiring_data.serialized_type = source_map.get_serialized_type(key)
            wiring_data.serialized_expected_type = json_encode(exp_type)
            data_list.append(wiring_data)
        return data_list


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


@define_bt_node(NodeConfig(options={}, inputs={}, outputs={}, max_children=1))
class Decorator(Node):
    """
    Base class for Decorator nodes.

    Decorators have exactly one child and somehow modify that child's
    output. Subclasses can add inputs, outputs and options, but never
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


@define_bt_node(NodeConfig(options={}, inputs={}, outputs={}, max_children=0))
class Leaf(Node):
    """
    Base class for leaf nodes in the tree.

    Leaf nodes have no children. Subclasses can define inputs, outputs
    and options, but never change `max_children`.
    """


@define_bt_node(NodeConfig(options={}, inputs={}, outputs={}, max_children=None))
class FlowControl(Node):
    """
    Base class for flow control nodes.

    Flow control nodes (mostly Sequence, Fallback and their derivatives)
    can have an unlimited number of children and each have a unique set
    of rules for when to tick which of their children.
    """


@define_bt_node(NodeConfig(options={}, inputs={}, outputs={}, max_children=0))
class IO(Node):
    """
    Base class for IO nodes in the tree.

    IO nodes have no children. Subclasses can define inputs, outputs
    and options, but never change `max_children`.
    """
