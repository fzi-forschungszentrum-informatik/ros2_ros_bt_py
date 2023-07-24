# Copyright 2023 FZI Forschungszentrum Informatik

from typing import Dict, Optional, List


class OptionRef(object):
    """
    Mark an input or output type as dependent on an option value.

    Can be used instead of an actual type in the maps passed to a
    :class:NodeConfig
    """

    def __init__(self, option_key):
        self.option_key = option_key

    def __repr__(self):
        return f"OptionRef(option_key={self.option_key!r})"

    def __eq__(self, other):
        return self.option_key == other.option_key

    def __ne__(self, other):
        return not self == other

    def __name__(self):
        return f"OptionRef(option_key={self.option_key!r})"


class NodeConfig(object):
    def __init__(
        self,
        options: Dict[str, type],
        inputs: Dict[str, type],
        outputs: Dict[str, type],
        max_children: Optional[int],
        optional_options: Optional[List[str]] = None,
        version: str = "",
        tags: Optional[List[str]] = None,
    ):
        """
        Describe the interface of a :class:ros_bt_py.node.Node .

        :type options Dict[str, type]
        :param options

        Map from option names to their types. Note that unlike `inputs`
        and `outputs`, option types can **not** use :class:OptionRef !

        :type inputs Dict[str, type]
        :param inputs:

        Map from input names to their types, or an :class:OptionRef
        object that points to the option key to take the type from.

        :type outputs Dict[str, type]
        :param outputs:

        Map from output names to their types, or an :class:OptionRef
        object that points to the option key to take the type from.

        :type max_children: int or None
        :param max_children:

        The maximum number of children this node type is allowed to
        have.  If this is None, the node type supports any amount of
        children.
        """
        self.inputs = inputs
        self.outputs = outputs
        self.options = options
        self.max_children = max_children

        if optional_options is None:
            optional_options = []
        self.optional_options = optional_options
        self.version = version

        if tags is None:
            tags = []
        self.tags = tags

    def __repr__(self):
        return (
            "NodeConfig(inputs=%r, outputs=%r, options=%r, max_children=%r,"
            " optional_options=%s, version=%s)"
            % (
                self.inputs,
                self.outputs,
                self.options,
                self.max_children,
                self.optional_options,
                self.version,
            )
        )

    def __eq__(self, other):
        return (
            self.inputs == other.inputs
            and self.outputs == other.outputs
            and self.options == other.options
            and self.max_children == other.max_children
            and self.optional_options == other.optional_options
            and self.version == other.version
        )

    def __ne__(self, other):
        return not self == other

    def extend(self, other):
        """
        Extend the input, output and option dicts with values from `other`.

        :raises: KeyError, ValueError
          If any of the dicts in `other` contains keys that already exist,
          raise `KeyError`. If `max_children` has a value different from ours,
          raise `ValueError`.
        """
        if self.max_children != other.max_children:
            raise ValueError(
                f"Mismatch in max_children: {self.max_children} vs {other.max_children}"
            )
        duplicate_inputs = []
        for key in other.inputs:
            if key in self.inputs:
                duplicate_inputs.append(key)
                continue
            self.inputs[key] = other.inputs[key]
        duplicate_outputs = []
        for key in other.outputs:
            if key in self.outputs:
                duplicate_outputs.append(key)
                continue
            self.outputs[key] = other.outputs[key]
        duplicate_options = []
        for key in other.options:
            if key in self.options:
                duplicate_options.append(key)
                continue
            self.options[key] = other.options[key]

        for optional_option in other.optional_options:
            if optional_option in self.optional_options:
                continue
            else:
                self.optional_options.append(optional_option)

        if duplicate_inputs or duplicate_outputs or duplicate_options:
            msg = "Duplicate keys: "
            keys_strings = []
            if duplicate_inputs:
                keys_strings.append(f"inputs: {str(duplicate_inputs)}")
            if duplicate_outputs:
                keys_strings.append(f"outputs: {str(duplicate_outputs)}")
            if duplicate_options:
                keys_strings.append(f"options: {str(duplicate_options)}")
            msg += ", ".join(keys_strings)
            raise KeyError(msg)
