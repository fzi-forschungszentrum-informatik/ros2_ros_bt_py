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

import abc
import array
import builtins
from copy import deepcopy
import json
import re

# TODO Include the import for `typing.Self` and apply it (check inline comments).
from typing import (
    Any,
    ClassVar,
    Generic,
    Iterable,
    Optional,
    Protocol,
    TypeGuard,
    TypeVar,
)
from typeguard import typechecked

from ros_bt_py.helpers import INT_LIMITS, FLOAT_LIMITS, INT_FLOAT_MAX
from ros_bt_py.ros_helpers import get_interface_name

import rosidl_runtime_py
import rosidl_runtime_py.utilities

from ros_bt_py.vendor.result import Result, Ok, Err

from ros_bt_py_interfaces.msg import NodeDataType

from example_interfaces import msg, srv, action


ANY = TypeVar("ANY")


class DataContainer(Generic[ANY], abc.ABC):
    """
    The common base class for all node io data types.

    Defines all interfaces to generically interact with data types,
    as well as basic implementations where applicable.
    """

    # This type_identifier has to be assigned a value in subclass definitions
    #   and should only be a class attribute.
    """The type identifier maps data types to :obj:`NodeDataType` messages"""
    type_identifier: ClassVar[int]

    allow_dynamic: bool
    allow_static: bool
    is_static: bool
    _value: Optional[ANY] = None
    _updated: bool

    @typechecked
    def __init__(
        self,
        allow_dynamic: bool = True,
        allow_static: bool = True,
        is_static: bool | None = None,
        value: Optional[ANY] = None,
    ):
        # This method calls `set_value` with the given value,
        #   so subclasses need to ensure that method is functional
        #   before calling `super().__init__`.
        # This also raises `ValueError` if the given value can't be assigned
        super().__init__()

        if not hasattr(self, "type_identifier"):
            raise RuntimeError(
                "All concrete implementations have to specify a type_identifier"
            )

        if not allow_dynamic and not allow_static:
            raise RuntimeError(
                "Need to allow either static or dynamic value assignment"
            )

        self.allow_dynamic = allow_dynamic
        self.allow_static = allow_static

        if is_static is None:
            self.is_static = self.allow_static
        else:
            self.is_static = is_static

        if self.is_static and not self.allow_static:
            raise RuntimeError("Cannot be static when static is not allowed")
        if not self.is_static and not self.allow_dynamic:
            raise RuntimeError("Cannot be dynamic when dynamic is not allowed")

        self.reset_updated()
        if value is not None:
            match self.set_value(value):
                case Err(e):
                    raise ValueError(e)
                case Ok(None):
                    pass

    def __eq__(self, other: object) -> bool:
        if not isinstance(other, DataContainer):
            return False
        if not self.is_compatible(other):
            return False
        if not other.is_compatible(self):
            return False
        if self.is_static != other.is_static:
            return False
        same_value: bool | Iterable[bool] = self._value == other._value
        # Since we potentially handle numpy arrays, we have to account for `==`
        #   returing an iterable of bools rather than a single bool
        if isinstance(same_value, Iterable):
            same_value = all(same_value)
        if not same_value:
            return False
        return True

    @classmethod
    @abc.abstractmethod
    @typechecked
    def _dict_from_msg(cls, msg: NodeDataType) -> Result[dict, str]:
        """
        Constructs a dictionary of parameters that can be used to initialize this data type.

        Subclasses should extend this to add their specific parameters.
        """
        return Ok(
            {
                "allow_dynamic": msg.allow_dynamic,
                "allow_static": msg.allow_static,
                "is_static": msg.is_static,
                "value": None,
            }
        )

    @classmethod
    @typechecked
    def from_msg(
        cls, msg: NodeDataType
    ) -> Result["DataContainer", str]:  # -> Result[Self, str]
        """
        Factory function that returns an instance of this data type,
        based on the given :obj:`NodeDataType` ROS message.

        This only verifies that the type identifier matches and does the final initialization step,
        the parameters for constructing the data type are handled by
        the helper function :obj:`self._dict_from_msg`.
        """
        if not hasattr(cls, "type_identifier"):
            raise NotImplementedError("Called on abstract base class")
        if msg.type_identifier != cls.type_identifier:
            return Err("Wrong type identifier")
        match cls._dict_from_msg(msg):
            case Err(e):
                return Err(e)
            case Ok(d):
                obj = cls(**d)
        return Ok(obj)

    @abc.abstractmethod
    @typechecked
    def is_compatible(
        self, other: "DataContainer"
    ) -> TypeGuard["DataContainer"]:  # TypeGuard[Self]
        """
        Check if the given container is compatible with this one,
        meaning that its constraints are at least as narrow as the ones in :obj:`self`.

        This is used to compare configs given on specific nodes
        with the baseline given on the class config.

        This can also serve as a type guard for :obj:`other`,
        because the checks performed are more strict than simple type equality.

        Subclasses should extend this with additional checks where applicable.
        """
        if not isinstance(other, self.__class__):
            return False
        if self.allow_dynamic and not other.allow_dynamic:
            return False
        if self.allow_static and not other.allow_static:
            return False
        # Don't compare `is_static`, since it serves as a default value not a constraint
        return True

    @abc.abstractmethod
    def serialize_type(self) -> NodeDataType:
        """
        Returns a serialized version of this data type as a :obj:`NodeDataType` ROS message.
        Subclasses should extend this if there are additional parameters to be added.
        """
        type_msg = NodeDataType(
            type_identifier=self.type_identifier,
            allow_dynamic=self.allow_dynamic,
            allow_static=self.allow_static,
            is_static=self.is_static,
        )
        return type_msg

    def get_runtime_type(self) -> "DataContainer":  # -> Self
        """
        Returns the own runtime type, which is used to determine whether two types
        have compatible values at runtime (is used to validate wirings between data types).

        The returned runtime type usually does NOT hold the value of the original type
        """
        type_msg = self.serialize_type()
        # At runtime we care whether the value IS static or not,
        #   so we overwrite `allow_dynamic` and `allow_static` accordingly
        type_msg.allow_dynamic = not type_msg.is_static
        type_msg.allow_static = type_msg.is_static
        # This `unwrap_or` is just an additional safety net,
        #   a sequence of serializing and deserializing should NEVER fail
        return self.from_msg(type_msg).unwrap_or(self)

    def has_value(self) -> bool:
        return self._value is not None

    @abc.abstractmethod
    @typechecked
    def set_value(self, value: ANY) -> Result[None, str]:
        """
        Sets the value of this data type, as well as the :obj:`updated` flag
        if the value differs from the previous one.

        Subclasses should validate and clean incoming values
        before calling :obj:`super().set_value` to assign them.
        """
        if self.is_static and self._updated:
            return Err("Static value was already assigned")
        # Only set `_updated` if the new value is different from the old
        has_changed: bool | Iterable[bool] = self._value != value
        # Since we potentially handle numpy arrays, we have to account for `==`
        #   returing an iterable of bools rather than a single bool
        if isinstance(has_changed, Iterable):
            has_changed = any(has_changed)
        self._updated = has_changed
        if has_changed:
            self._value = deepcopy(value)
        return Ok(None)

    @typechecked
    def get_value(self) -> Result[ANY, None]:
        """
        Returns an :obj:`Ok` holding the value or
        an empty :obj:`Err` if the value is :obj:`None`.
        """
        if self._value is None:
            return Err(None)
        return Ok(deepcopy(self._value))

    @typechecked
    def get_value_as(self, type_: type[ANY]) -> Result[ANY, Any]:
        """
        Coerces the stored value to the given :obj:`type_` if possible.
        Returns :obj:`Ok` if the type matches and :obj:`Err` if it doesn't,
        but both results wrap the same value (or :obj:`None` in case of :obj:`Err`)
        """
        match self.get_value():
            case Err(None):
                return Err(None)
            case Ok(v):
                value = v
        if isinstance(value, type_):
            return Ok(value)
        return Err(value)

    def reset_value(self):
        if not self.is_static:
            self._value = None

    def is_updated(self) -> bool:
        return self._updated

    def flag_updated(self):
        """
        Set the :obj:`updated` flag to :obj:`True`
        """
        self._updated = True

    def reset_updated(self):
        """
        Set the :obj:`updated` flag to :obj:`False`
        """
        self._updated = False

    @typechecked
    def serialize_value(self) -> str:
        """
        Checks whether this type has a set value. If not, just return an empty string.
        Calls the :obj:`self._serialize_value` helper function to get a json serializable
        representation of the value, then pass that to :obj:`json.dumps`
        """
        match self.get_value():
            case Err(None):
                return ""
            case Ok(v):
                value = v
        return json.dumps(
            obj=self._serialize_value(value),
            skipkeys=True,
            default=lambda _: "",
        )

    @typechecked
    def _serialize_value(self, value: ANY) -> Any:
        """
        Subclasses overwrite this to include custom serialization steps when necessary.
        The output should be json-serializeable, but not serialized yet.
        The wrapping method calls :obj:`json.dumps` as a final step.
        """
        return value

    @typechecked
    def deserialize_value(self, ser_value: str) -> Result[None, str]:
        """
        First pass the given serialized value to :obj:`json.loads`,
        then to the :obj:`self._deserialize_value` helper function,
        and finally to :obj:`self.set_value`.
        """
        value = json.loads(ser_value)
        return self._deserialize_value(value).and_then(lambda val: self.set_value(val))

    @typechecked
    def _deserialize_value(self, value: Any) -> Result[ANY, str]:
        """
        Subclasses overwrite this to include custom deserialization steps when necessary.
        The input has already been passed through :obj:`json.loads` by the wrapping function.
        """
        return Ok(value)


# This list gets populated through the `register_io_type` decorator,
#   and is used to identify the type of any `NodeDataType` message.
CONCRETE_IO_TYPES: list[type[DataContainer]] = []


CONTAINER = TypeVar("CONTAINER", bound=DataContainer)


@typechecked
def register_io_type(cls: type[CONTAINER]) -> type[CONTAINER]:
    """
    This decorator is used to register concrete data types,
    which allows them to be found when parsing a :obj:`NodeDataType`
    message with :obj:`get_iotype_for_msg`.
    """
    if not hasattr(cls, "type_identifier"):
        raise RuntimeError("Registered IO types require a type identifier")
    CONCRETE_IO_TYPES.append(cls)
    return cls


@typechecked
def get_iotype_for_msg(msg: NodeDataType) -> Result[DataContainer, str]:
    """
    Parses a :obj:`NodeDataType` message to a data type class
    that was registered with :obj:`register_io_type`.
    """
    for io_type in CONCRETE_IO_TYPES:
        match io_type.from_msg(msg):
            case Err(_):
                continue  # Try all io types
            case Ok(io):
                return Ok(io)
    return Err("There is no IO type matching this identifier.")


# The inheritence here is only pro-forma to typecheck the constructor.
#   It is recommended that implementations specify a container explicitly
#   E.g. `class ValueType[V](TypeContainerMixin, DataContainer[V])`
class TypeContainerMixin(DataContainer):
    """
    Mixin class to signal that a data type is a valid target for reference types.

    This forces the values to always be static and defines the :obj:`get_value_field` interface.
    """

    # Update the defaults and verify that a type container doesn't allow dynamic values
    @typechecked
    def __init__(
        self,
        allow_dynamic=False,
        allow_static=True,
        *args,
        **kwargs,
    ) -> None:
        if allow_dynamic:
            raise RuntimeError(
                "Type containers cannot be dynamic, since they're (potentially) reference targets."
            )
        super().__init__(
            allow_dynamic=allow_dynamic,
            allow_static=allow_static,
            *args,
            **kwargs,
        )

    @abc.abstractmethod
    def get_value_field(self) -> Result[DataContainer, None]:
        raise NotImplementedError("Can't get value field for base class")


BUILTIN = TypeVar("BUILTIN", bool, int, float, str, list, dict, bytes, object)


class BuiltinContainer(DataContainer[BUILTIN]):
    """
    Common base class for all data type classes that correspond to primitive data types.
    """

    _type: type[BUILTIN]
    _value: BUILTIN

    @typechecked
    def set_value(self, value: BUILTIN) -> Result[None, str]:
        if not isinstance(value, self._type):
            return Err(f"Given value {value} isn't of type {self._type}")
        return super().set_value(value)


@register_io_type
class BoolType(BuiltinContainer[bool]):
    """
    This type holds a simple boolean value
    """

    type_identifier = NodeDataType.BOOL_TYPE
    _type = bool
    _value = False

    @classmethod
    def _dict_from_msg(cls, msg: NodeDataType) -> Result[dict, str]:
        # Nothing to add for bool
        return super()._dict_from_msg(msg)

    def is_compatible(
        self, other: DataContainer
    ) -> TypeGuard["BoolType"]:  # -> TypeGuard[Self]
        # Nothing to add for bool
        return super().is_compatible(other)

    def serialize_type(self) -> NodeDataType:
        # Nothing to add for bool
        return super().serialize_type()


NUM = TypeVar("NUM", int, float)


class NumericContainer(BuiltinContainer[NUM]):
    """
    Base class for numeric type classes, that are constrained by a minimum and maximum value.
    """

    min_value: NUM
    max_value: NUM
    lower_limit: NUM
    upper_limit: NUM

    @typechecked
    def __init__(
        self,
        min_value: Optional[NUM] = None,
        max_value: Optional[NUM] = None,
        *args,
        **kwargs,
    ) -> None:
        self.min_value = min_value if min_value is not None else self.lower_limit
        self.max_value = max_value if max_value is not None else self.upper_limit

        if self.min_value > self.max_value:
            raise RuntimeError(
                f"Given minimum {self.min_value} is larger than maximum {self.max_value}"
            )

        super().__init__(*args, **kwargs)

        if self.is_static and not self.is_updated():
            # Check boundaries and adjust automatic default
            if self.min_value > 0:
                self._value = self.min_value
            if self.max_value < 0:
                self._value = self.max_value

    @typechecked
    def set_value(self, value: NUM) -> Result[None, str]:
        if value < self.min_value:
            return Err(f"Given value {value} is smaller than minimum {self.min_value}")
        if value > self.max_value:
            return Err(f"Given value {value} is larger than maximum {self.max_value}")
        return super().set_value(value)

    def is_compatible(
        self, other: DataContainer
    ) -> TypeGuard["NumericContainer"]:  # -> TypeGuard[Self]
        if not super().is_compatible(other):
            return False
        if other.min_value < self.min_value:
            return False
        if other.max_value > self.max_value:
            return False
        return True

    def serialize_type(self) -> NodeDataType:
        type_msg = super().serialize_type()
        type_msg.min_value = str(self.min_value)
        type_msg.max_value = str(self.max_value)
        return type_msg


@register_io_type
class IntType(NumericContainer[int]):
    """
    This type holds an integer value.
    """

    type_identifier = NodeDataType.INT_TYPE
    _type = int
    _value = 0
    lower_limit = INT_LIMITS["int64"][0]
    upper_limit = INT_LIMITS["uint64"][1]

    @classmethod
    def _dict_from_msg(cls, msg: NodeDataType) -> Result[dict, str]:
        match super()._dict_from_msg(msg):
            case Err(e):
                return Err(e)
            case Ok(c):
                config = c
        config["min_value"] = int(msg.min_value)
        config["max_value"] = int(msg.max_value)
        return Ok(config)


@register_io_type
class FloatType(NumericContainer[float]):
    """
    This type holds a floating point value.
    """

    type_identifier = NodeDataType.FLOAT_TYPE
    _type = float
    _value = 1.2
    lower_limit, upper_limit = FLOAT_LIMITS["double"]

    @classmethod
    def _dict_from_msg(cls, msg: NodeDataType) -> Result[dict, str]:
        match super()._dict_from_msg(msg):
            case Err(e):
                return Err(e)
            case Ok(c):
                config = c
        config["min_value"] = float(msg.min_value)
        config["max_value"] = float(msg.max_value)
        return Ok(config)

    def set_value(self, value: float | int) -> Result[None, str]:
        # Silently convert int to float
        if isinstance(value, int):
            value = float(value)
        return super().set_value(value)


STRING = TypeVar("STRING", str, bytes)


class StringContainer(BuiltinContainer[STRING]):
    """
    Common base class for all "string-like" data types,
    which are optionally constrained by a maximum length.

    Alternatively they can be constrained by a set of valid value options.
    """

    max_length: int = INT_FLOAT_MAX
    valid_values: Optional[list[str]]

    def __init__(
        self,
        max_length: Optional[int] = None,
        valid_values: Optional[list[str]] = None,
        *args,
        **kwargs,
    ) -> None:
        if max_length is not None:
            self.max_length = max_length
        self.valid_values = valid_values

        super().__init__(*args, **kwargs)

    @classmethod
    def _dict_from_msg(cls, msg: NodeDataType) -> Result[dict, str]:
        match super()._dict_from_msg(msg):
            case Err(e):
                return Err(e)
            case Ok(c):
                config = c
        config["max_length"] = msg.string_max_length
        if len(msg.serialized_value_options) > 0:
            config["valid_values"] = msg.serialized_value_options
        return Ok(config)

    @typechecked
    def set_value(self, value: STRING) -> Result[None, str]:
        if len(value) > self.max_length:
            return Err(f"Length of {value} exceeds maximum of {self.max_length}")
        if self.valid_values is not None:
            if value not in self.valid_values:
                return Err(
                    f"Value {value} is not a valid value of ({self.valid_values})"
                )
        return super().set_value(value)

    def is_compatible(
        self, other: DataContainer
    ) -> TypeGuard["StringContainer"]:  # -> TypeGuard[Self]
        if not super().is_compatible(other):
            return False
        if other.max_length > self.max_length:
            return False
        if self.valid_values is not None:
            if other.valid_values is None:
                return False
            for val in self.valid_values:
                if val not in other.valid_values:
                    return False
        return True

    def serialize_type(self) -> NodeDataType:
        type_msg = super().serialize_type()
        type_msg.string_max_length = self.max_length
        if self.valid_values is not None:
            type_msg.serialized_value_options = self.valid_values
        return type_msg


@register_io_type
class StringType(StringContainer[str]):
    """
    This type holds a string value.
    """

    type_identifier = NodeDataType.STRING_TYPE
    _type = str
    _value = ""


@register_io_type
class PathType(StringContainer[str]):
    """
    This type holds a path uri, which has to start with 'file://' or 'package://'
    """

    type_identifier = NodeDataType.PATH_TYPE
    _type = str
    _value = ""

    @typechecked
    def set_value(self, value: str) -> Result[None, str]:
        if not value.startswith("file://") and not value.startswith("package://"):
            return Err(f"Value {value} is not a valid file or package uri")
        return super().set_value(value)


@register_io_type
class BytesType(StringContainer[bytes]):
    """
    This type holds a bytes object.
    The length restriction is set to 1 by default,
    since the bytes type is mostly used to fill 'byte' fields in ROS messages,
    which only take one byte.
    Bytes are serializes as hex strings.
    """

    type_identifier = NodeDataType.BYTES_TYPE
    _type = bytes
    _value = b"\x00"

    max_length = 1

    @typechecked
    def _serialize_value(self, value: bytes) -> str:
        return value.hex()

    @typechecked
    def _deserialize_value(self, ser_value: str) -> Result[bytes, str]:
        try:
            return Ok(bytes.fromhex(ser_value))
        except ValueError:
            return Err(f"Given string {ser_value} isn't valid hexcode")


ITER = TypeVar("ITER", list, dict)


class IterableContainer(BuiltinContainer[ITER]):
    """
    Note that the static/dynamic attributes on element types are ignored,
    those have to be specified on the iterable itself.

    If the element type is omitted, all kinds of values are accepted,
    but value types that can't be serialized as json will silently be
    replaced with "" in any serialized output.

    Iterables can also constrained by a maximum length,
    with a boolean flag to make that limit 'strict',
    which means the iterable has to match that maximum length exactly.
    """

    _element_type: DataContainer = IntType()
    max_length: int = INT_FLOAT_MAX
    strict_length: bool = False

    @typechecked
    def __init__(
        self,
        element_type: Optional[DataContainer] = None,
        max_length: Optional[int] = None,
        strict_length: Optional[bool] = None,
        *args,
        **kwargs,
    ) -> None:

        if element_type is not None:
            self._element_type = element_type

        # Force element_type to accept dynamic values
        self._element_type.allow_dynamic = True
        self._element_type.is_static = False

        if max_length is not None:
            self.max_length = max_length
        if strict_length is not None:
            self.strict_length = strict_length

        super().__init__(*args, **kwargs)

    @classmethod
    def _dict_from_msg(cls, msg: NodeDataType) -> Result[dict, str]:
        match super()._dict_from_msg(msg):
            case Err(e):
                return Err(e)
            case Ok(c):
                config = c
        if (
            len(msg.value_type_identifier) == 0
            or len(msg.iterable_max_length) == 0
            or len(msg.iterable_strict_length) == 0
        ):
            return Err(
                f"Message {msg} is for an iterable but doesn't specify proper constraints"
            )
        inner_msg = deepcopy(msg)
        inner_msg.type_identifier = msg.value_type_identifier[0]
        inner_msg.value_type_identifier = msg.value_type_identifier[1:]
        inner_msg.iterable_max_length = msg.iterable_max_length[1:]
        inner_msg.iterable_strict_length = msg.iterable_strict_length[1:]  # type: ignore
        match get_iotype_for_msg(inner_msg):
            case Err(e):
                return Err(e)
            case Ok(c):
                config["element_type"] = c
        config["max_length"] = msg.iterable_max_length[0]
        config["strict_length"] = msg.iterable_strict_length[0]  # type: ignore
        return Ok(config)

    def is_compatible(
        self, other: DataContainer
    ) -> TypeGuard["IterableContainer"]:  # -> TypeGuard[Self]
        if not super().is_compatible(other):
            return False
        if other.max_length > self.max_length:
            return False
        if self.strict_length:
            if not other.strict_length:
                return False
            if other.max_length != self.max_length:
                return False
        return self._element_type.is_compatible(other._element_type)

    def serialize_type(self) -> NodeDataType:
        type_msg = super().serialize_type()
        # Overlay msg of iterable type on msg of element type
        inner_type_msg = self._element_type.serialize_type()
        inner_type_msg.allow_dynamic = type_msg.allow_dynamic
        inner_type_msg.allow_static = type_msg.allow_static
        inner_type_msg.is_static = type_msg.is_static
        inner_type_msg.value_type_identifier.append(inner_type_msg.type_identifier)
        inner_type_msg.type_identifier = type_msg.type_identifier
        inner_type_msg.iterable_max_length.append(self.max_length)
        inner_type_msg.iterable_strict_length.append(self.strict_length)  # type: ignore
        return inner_type_msg

    @abc.abstractmethod
    @typechecked
    def set_value(self, value: ITER) -> Result[None, str]:
        if len(value) > self.max_length:
            return Err(
                f"Value {value} has more items than the limit of {self.max_length}"
            )
        if self.strict_length and len(value) < self.max_length:
            return Err(
                f"Value {value} has fewer items than the limit of {self.max_length}"
            )
        return super().set_value(value)


@register_io_type
class ListType(IterableContainer[list[Any]]):
    """
    This type holds a list of values.
    """

    type_identifier = NodeDataType.LIST_TYPE
    _type = list
    _value = []

    @typechecked
    def set_value(self, value: list | array.array) -> Result[None, str]:
        if isinstance(value, array.array):
            value = list(value)
        for item in value:
            match self._element_type.set_value(item):
                case Err(e):
                    return Err(e)
                case Ok(None):
                    pass
        return super().set_value(value)

    @typechecked
    def _serialize_value(self, value: list[Any]) -> list[Any]:
        serialized_list = []
        for item in value:
            serialized_list.append(self._element_type._serialize_value(item))
        return serialized_list

    @typechecked
    def _deserialize_value(self, ser_value: list[Any]) -> Result[list[Any], str]:
        value = []
        for item in ser_value:
            match self._element_type._deserialize_value(item):
                case Err(e):
                    return Err(e)
                case Ok(v):
                    value.append(v)
        return Ok(value)


@register_io_type
class DictType(IterableContainer[dict[str, Any]]):
    """
    This type holds a dict of values.
    The keys of a dict will always be coerced by using :obj:`str(...)`.
    """

    type_identifier = NodeDataType.DICT_TYPE
    _type = dict
    _value = {}

    @typechecked
    def set_value(self, value: dict) -> Result[None, str]:
        cleaned_keys = {str(k): v for k, v in value.items()}
        for item in cleaned_keys.values():
            match self._element_type.set_value(item):
                case Err(e):
                    return Err(e)
                case Ok(None):
                    pass
        return super().set_value(cleaned_keys)

    @typechecked
    def _serialize_value(self, value: dict[str, Any]) -> dict[str, str]:
        serialized_dict: dict[str, str] = {}
        for key, item in value.items():
            serialized_dict[key] = self._element_type._serialize_value(item)
        return serialized_dict

    @typechecked
    def _deserialize_value(
        self, ser_value: dict[str, Any]
    ) -> Result[dict[str, Any], str]:
        value = {}
        for key, item in ser_value.items():
            match self._element_type._deserialize_value(item):
                case Err(e):
                    return Err(e)
                case Ok(v):
                    value[key] = v
        return Ok(value)


# Constants that define the structure of type values for builtin types
IDENTIFIER_KEY = "type_identifier"
ELEMENT_KEY = "element_type"
MESSAGE_KEY = "ros_msg_type"
GENERIC_TYPE_MAP: dict[type, dict] = {
    bool: {IDENTIFIER_KEY: NodeDataType.BOOL_TYPE},
    int: {
        IDENTIFIER_KEY: NodeDataType.INT_TYPE,
        "min_value": INT_LIMITS["int64"][0],
        "max_value": INT_LIMITS["int64"][1],
    },
    float: {
        IDENTIFIER_KEY: NodeDataType.FLOAT_TYPE,
        "min_value": FLOAT_LIMITS["double"][0],
        "max_value": FLOAT_LIMITS["double"][1],
    },
    str: {
        IDENTIFIER_KEY: NodeDataType.STRING_TYPE,
        "max_length": INT_FLOAT_MAX,
    },
    bytes: {
        IDENTIFIER_KEY: NodeDataType.BYTES_TYPE,
        "max_length": 1,
    },
    list: {
        IDENTIFIER_KEY: NodeDataType.LIST_TYPE,
        "max_length": INT_FLOAT_MAX,
        "strict_length": False,
        ELEMENT_KEY: {IDENTIFIER_KEY: NodeDataType.BOOL_TYPE},
    },
    dict: {
        IDENTIFIER_KEY: NodeDataType.DICT_TYPE,
        "max_length": INT_FLOAT_MAX,
        "strict_length": False,
        ELEMENT_KEY: {IDENTIFIER_KEY: NodeDataType.BOOL_TYPE},
    },
    # `object` here is a placeholder for ROS messages
    object: {
        IDENTIFIER_KEY: NodeDataType.ROS_INTERFACE_VALUE,
        MESSAGE_KEY: "example_interfaces/msg/Empty",
    },
}


def match_dict_to_valid_types(value: dict, valid_types: list) -> Result[None, str]:
    for option in valid_types:
        option_value = GENERIC_TYPE_MAP[option]
        if option_value[IDENTIFIER_KEY] != value[IDENTIFIER_KEY]:
            continue
        if option_value.keys() == value.keys():
            if ELEMENT_KEY not in value.keys():
                return Ok(None)
            # Recursively check nested dicts
            return match_dict_to_valid_types(value[ELEMENT_KEY], valid_types)
        return Err(f"Dict {value} doesn't match template {option_value}")
    return Err(f"Identifier in dict {value} is invalid")


@typechecked
def get_iotype_for_dict(value_dict: dict) -> Result[DataContainer, str]:
    """
    Constructs the data type from a dictionary of type parameters.
    See also :obj:`GENERIC_TYPE_MAP`
    """
    try:
        if value_dict[IDENTIFIER_KEY] == NodeDataType.ROS_INTERFACE_VALUE:
            return Ok(
                RosMessageType(
                    message_type=rosidl_runtime_py.utilities.get_message(
                        value_dict[MESSAGE_KEY]
                    )
                )
            )
        io_class = None
        for io_type in CONCRETE_IO_TYPES:
            if io_type.type_identifier == value_dict[IDENTIFIER_KEY]:
                io_class = io_type
        if io_class is None:
            return Err("There is no IO type matching this identifier.")
        args_dict = deepcopy(value_dict)
        args_dict.pop(IDENTIFIER_KEY)
        if ELEMENT_KEY in args_dict:
            match get_iotype_for_dict(args_dict[ELEMENT_KEY]):
                case Err(e):
                    return Err(e)
                case Ok(c):
                    args_dict[ELEMENT_KEY] = c
        io_instance = io_class(**args_dict)
        return Ok(io_instance)
    except (KeyError, TypeError) as e:
        return Err(str(e))


@typechecked
def serialize_class(cls: type) -> str:
    """
    Serialize builtin types
    """
    return cls.__name__


@typechecked
def deserialize_class(ser_cls: str) -> Result[type, str]:
    """
    Deserialize builtin types
    """
    cls = getattr(builtins, ser_cls, None)
    if cls is None:
        return Err(f"Type {ser_cls} can't be found")
    return Ok(cls)


@typechecked
def serialize_type_map_value(val: dict) -> dict:
    """
    Helper function to serialize type values.
    """
    value_dict = deepcopy(val)
    # Coerce int & float values to string to avoid loss of precision
    if "min_value" in value_dict.keys():
        value_dict["min_value"] = str(value_dict["min_value"])
    if "max_value" in value_dict.keys():
        value_dict["max_value"] = str(value_dict["max_value"])
    if ELEMENT_KEY in value_dict.keys():
        value_dict[ELEMENT_KEY] = serialize_type_map_value(value_dict[ELEMENT_KEY])
    return value_dict


@typechecked
def deserialize_type_map_value(val: dict) -> dict:
    """
    Helper function to deserialize type values.
    """
    value_dict = deepcopy(val)
    if value_dict[IDENTIFIER_KEY] == NodeDataType.INT_TYPE:
        converter = int
    elif value_dict[IDENTIFIER_KEY] == NodeDataType.FLOAT_TYPE:
        converter = float
    else:
        if ELEMENT_KEY in value_dict.keys():
            value_dict[ELEMENT_KEY] = deserialize_type_map_value(
                value_dict[ELEMENT_KEY]
            )
        return value_dict
    # Parse int & float values from string to avoid loss of precision
    if "min_value" in value_dict.keys():
        value_dict["min_value"] = converter(value_dict["min_value"])
    if "max_value" in value_dict.keys():
        value_dict["max_value"] = converter(value_dict["max_value"])
    return value_dict


@typechecked
def serialize_type_map(keys: list[type]) -> list[str]:
    """
    Serialize the valid value list for builtin types.
    """
    ser_list: list[str] = []
    for key in keys:
        if key not in GENERIC_TYPE_MAP.keys():
            continue
        type_dict = {
            "type": serialize_class(key),
            "value": serialize_type_map_value(GENERIC_TYPE_MAP[key]),
        }
        ser_list.append(json.dumps(type_dict))
    return ser_list


@register_io_type
class GenericType(TypeContainerMixin, BuiltinContainer[dict]):
    """
    This holds a type value from the :obj:`GENERIC_TYPE_MAP` keys,
    which correspond to data types inheriting from :obj:`BuiltinContainer`.

    The list of valid types can optionally be constrained by supplying a list of builtin types.
    """

    type_identifier = NodeDataType.GENERIC_TYPE
    _type = dict
    valid_types: list[type]

    @typechecked
    def __init__(
        self,
        valid_types: list[type] = list(GENERIC_TYPE_MAP.keys()),
        *args,
        **kwargs,
    ) -> None:
        for type_ in valid_types:
            if type_ not in GENERIC_TYPE_MAP.keys():
                raise RuntimeError(f"Type {type_} is not usable as an IO type")
        self.valid_types = valid_types
        self._value = GENERIC_TYPE_MAP[valid_types[0]]
        super().__init__(*args, **kwargs)

    @classmethod
    def _dict_from_msg(cls, msg: NodeDataType) -> Result[dict, str]:
        match super()._dict_from_msg(msg):
            case Err(e):
                return Err(e)
            case Ok(c):
                config = c
        valid_types: list[type] = []
        for option in msg.serialized_value_options:
            option_dict = json.loads(option)
            match deserialize_class(option_dict["type"]):
                case Err(_):
                    continue
                case Ok(c):
                    valid_types.append(c)
        config["valid_types"] = valid_types
        return Ok(config)

    @typechecked
    def set_value(self, value: dict) -> Result[None, str]:
        if IDENTIFIER_KEY not in value.keys():
            return Err(f"Dict {value} doesn't include an identifier")
        match match_dict_to_valid_types(value, self.valid_types):
            case Err(e):
                return Err(e)
            case Ok(None):
                super().set_value(value)
                return Ok(None)

    def is_compatible(
        self, other: DataContainer
    ) -> TypeGuard["GenericType"]:  # -> TypeGuard[Self]
        if not super().is_compatible(other):
            return False
        for type_elem in other.valid_types:
            if type_elem not in self.valid_types:
                return False
        return True

    def serialize_type(self) -> NodeDataType:
        type_msg = super().serialize_type()
        type_msg.serialized_value_options = serialize_type_map(self.valid_types)
        return type_msg

    def _serialize_value(self, value: dict) -> dict:
        return serialize_type_map_value(value)

    def _deserialize_value(self, ser_value: dict) -> Result[dict, str]:
        return Ok(deserialize_type_map_value(ser_value))

    @typechecked
    def get_value_field(self) -> Result[DataContainer, None]:
        return (
            self.get_value()
            .and_then(lambda val: get_iotype_for_dict(val))
            .map_err(lambda _: None)
        )


class RosContainer(DataContainer):
    """
    Common base class for all data types related to ROS components (topics, services, actions).

    These types are further identified by the kind of interface they refer to,
    as well as an interface id to connect for example the name and type fields of the same service.
    """

    # This `interface_kind` has to be assigned a value in subclass definitions
    #   and should ONLY be a class attribute
    interface_kind: ClassVar[int]
    interface_id: int

    @typechecked
    def __init__(
        self,
        interface_id=0,
        *args,
        **kwargs,
    ) -> None:
        if not hasattr(self, "interface_kind"):
            raise RuntimeError(
                "All concrete implementations have to specify a kind of ROS interface"
            )

        super().__init__(*args, **kwargs)

        self.interface_id = interface_id

    @classmethod
    def _dict_from_msg(cls, msg: NodeDataType) -> Result[dict, str]:
        match super()._dict_from_msg(msg):
            case Err(e):
                return Err(e)
            case Ok(c):
                config = c
        config["interface_id"] = msg.interface_id
        return Ok(config)

    @classmethod
    @typechecked
    def from_msg(
        cls, msg: NodeDataType
    ) -> Result["RosContainer", str]:  # -> Result[Self, str]
        if not hasattr(cls, "interface_kind"):
            raise NotImplementedError("Called on abstract base class")
        if msg.ros_interface_kind != cls.interface_kind:
            return Err("Wrong kind of ROS interface")
        return super().from_msg(msg)

    def is_compatible(
        self, other: DataContainer
    ) -> TypeGuard["RosContainer"]:  # -> TypeGuard[Self]
        if not super().is_compatible(other):
            return False
        return self.interface_id == other.interface_id

    def serialize_type(self) -> NodeDataType:
        type_msg = super().serialize_type()
        type_msg.ros_interface_kind = self.interface_kind
        type_msg.interface_id = self.interface_id
        return type_msg


class RosNameContainer(RosContainer, BuiltinContainer[str]):
    """
    Common base class for ROS interface names.
    Also inherits from :obj:`BuiltinContainer` to include all necessary functionality.
    """

    type_identifier = NodeDataType.ROS_INTERFACE_NAME
    _value = "/foo"
    _type = str


@register_io_type
class RosTopicName(RosNameContainer):
    """
    Holds a ROS topic name as a string.
    """

    interface_kind = NodeDataType.ROS_TOPIC


@register_io_type
class RosServiceName(RosNameContainer):
    """
    Holds a ROS service name as a string.
    """

    interface_kind = NodeDataType.ROS_SERVICE


@register_io_type
class RosActionName(RosNameContainer):
    """
    Holds a ROS action name as a string.
    """

    interface_kind = NodeDataType.ROS_ACTION


class RosMessage(Protocol):
    """
    Structural base class for all ROS messages
    """

    @classmethod
    def get_fields_and_field_types(cls) -> dict[str, str]:
        raise NotImplementedError


@register_io_type
class RosMessageType(RosContainer):
    """
    Holds the values for a given ROS message, which has to be specified on :obj:`init`.
    """

    type_identifier = NodeDataType.ROS_INTERFACE_VALUE
    interface_kind = NodeDataType.ROS_TOPIC

    # This message_type is set by the factory function
    message_type: type[RosMessage]

    def __init__(self, message_type: type[RosMessage], *args, **kwargs) -> None:
        self.message_type = message_type
        self._value = message_type()
        super().__init__(*args, **kwargs)

    @classmethod
    def _dict_from_msg(cls, msg: NodeDataType) -> Result[dict, str]:
        try:
            message_type = rosidl_runtime_py.utilities.get_interface(msg.ros_msg_type)
            match super()._dict_from_msg(msg):
                case Err(e):
                    return Err(e)
                case Ok(d):
                    config = d
            config["message_type"] = message_type
            return Ok(config)
        except AttributeError:
            return Err(f"Cannot find message type '{msg.ros_msg_type}'")

    def is_compatible(
        self, other: DataContainer
    ) -> TypeGuard["RosMessageType"]:  # -> TypeGuard[Self]
        if not super().is_compatible(other):
            return False
        # This is basically an == check, since ROS types don't have inheritance.
        #   but we still use `issubclass` since it is a sufficient condition.
        return issubclass(other.message_type, self.message_type)

    def serialize_type(self) -> NodeDataType:
        type_msg = super().serialize_type()
        type_msg.ros_msg_type = get_interface_name(self.message_type)
        return type_msg

    def set_value(self, value: Any) -> Result[None, str]:
        if not isinstance(value, self.message_type):
            return Err(
                f"Value {value} is not the proper ROS message {self.message_type}"
            )
        return super().set_value(value)

    def _serialize_value(self, value: Any) -> dict[str, Any]:
        match self.get_element_fields():
            case Err(_):
                return {}
            case Ok(d):
                field_dict = d
        out_dict = {}
        for field_name, field_type in field_dict.items():
            out_dict[field_name] = field_type._serialize_value(
                getattr(value, field_name, None)
            )
        return out_dict

    @typechecked
    def _deserialize_value(self, ser_value: dict[str, Any]) -> Result[Any, str]:
        match self.get_element_fields():
            case Err(e):
                return Err(e)
            case Ok(d):
                field_dict = d
        value = self.message_type()
        for field_name, field_value in ser_value.items():
            if field_name in field_dict.keys():
                field_type = field_dict[field_name]
                match field_type._deserialize_value(field_value):
                    case Err(e):
                        return Err(e)
                    case Ok(v):
                        setattr(value, field_name, v)
            else:
                return Err(
                    f"Key {field_name} does not exist on message type {self.message_type}"
                )
        return Ok(value)

    def get_element_fields(self) -> Result[dict[str, DataContainer], str]:
        out_dict = {}
        for (
            field_name,
            field_type,
        ) in self.message_type.get_fields_and_field_types().items():
            match get_message_field_io_type(field_type):
                case Err(e):
                    return Err(e)
                case Ok(c):
                    out_dict[field_name] = c
        return Ok(out_dict)


class RosTypeContainer(TypeContainerMixin, RosContainer):
    """
    Common base class for all ROS message types.

    Defines an abstract :obj:`self._validate` method which checks
    if the given type is actually a ROS message.
    """

    type_identifier = NodeDataType.ROS_INTERFACE_TYPE

    # Adapt defaults to common use case of type specification (static only)
    def __init__(
        self,
        allow_dynamic=False,
        allow_static=True,
        *args,
        **kwargs,
    ) -> None:
        super().__init__(
            allow_dynamic=allow_dynamic,
            allow_static=allow_static,
            *args,
            **kwargs,
        )

    @staticmethod
    @abc.abstractmethod
    def _validate(value: type) -> bool:
        raise NotImplementedError("Can't validate on a base class")

    @typechecked
    def set_value(self, value: type) -> Result[None, str]:
        if not self._validate(value):
            return Err(f"Value {value} is not a matching ROS type")
        return super().set_value(value)

    @typechecked
    def _serialize_value(self, value: type) -> str:
        return get_interface_name(value)

    @typechecked
    def _deserialize_value(self, ser_value: str) -> Result[Any, str]:
        try:
            msg = rosidl_runtime_py.utilities.get_interface(ser_value)
        except ValueError:
            return Err(
                f"Serialized value {ser_value} does not point to a valid ROS interface"
            )
        return Ok(msg)

    @typechecked
    def get_value_field(self) -> Result[DataContainer, None]:
        match self.get_value():
            case Err(None):
                return Err(None)
            case Ok(v):
                value = v
        return Ok(RosMessageType(message_type=value))


@register_io_type
class RosTopicType(RosTypeContainer):
    """
    This type holds message types of ROS topics.

    Note that this validation also accepts component messages
    like :obj:`Service_Request` or :obj:`Action_Goal`,
    since they're fully fledged message classes.
    The interface type just indicates that we are not looking for those.
    """

    interface_kind = NodeDataType.ROS_TOPIC
    _value = msg.Empty

    @staticmethod
    @typechecked
    def _validate(value: type) -> bool:
        return rosidl_runtime_py.utilities.is_message(value)


@register_io_type
class RosServiceType(RosTypeContainer):
    """
    This type holds message types of ROS services.
    """

    interface_kind = NodeDataType.ROS_SERVICE
    _value = srv.Trigger

    @staticmethod
    @typechecked
    def _validate(value: type) -> bool:
        return rosidl_runtime_py.utilities.is_service(value)


@register_io_type
class RosActionType(RosTypeContainer):
    """
    This type holds message types of ROS actions.
    """

    interface_kind = NodeDataType.ROS_ACTION
    _value = action.Fibonacci

    @staticmethod
    @typechecked
    def _validate(value: type) -> bool:
        return rosidl_runtime_py.utilities.is_action(value)


@register_io_type
class RosComponentType(RosTypeContainer):
    """
    This type holds component message types.

    This behaves similar to :obj:`RosTopicType`,
    except that the interface type indicates that we also want to allow
    interface components like :obj:`Service_Request` or :obj:`Action_Goal`.
    """

    interface_kind = NodeDataType.ROS_COMPONENT
    _value = msg.Empty

    @staticmethod
    @typechecked
    def _validate(value: type) -> bool:
        return rosidl_runtime_py.utilities.is_message(value)


class ReferenceContainer(DataContainer[Any]):
    """
    Common base class for all reference types.
    Reference types imitate other data types based on the referenced type value.
    The data type being imitated is stored as :obj:`self._inner_type`.

    Defines additional functions necessary to fully initialize these reference types.
    """

    _value: None = None
    _reference: str
    _container_map: dict[str, DataContainer[Any]] = {}
    _inner_type: Optional[DataContainer[Any]] = None

    @typechecked
    def __init__(
        self,
        reference: str,
        allow_dynamic: bool = True,
        allow_static: bool = True,
        is_static: bool | None = None,
    ) -> None:
        super().__init__(
            allow_dynamic=allow_dynamic,
            allow_static=allow_static,
            is_static=is_static,
            value=None,
        )
        self._reference = reference

    def _set_inner_type(self) -> Result[None, str]:
        """
        Sets the final type of this reference based on the referenced type value.
        """
        if self._reference not in self._container_map:
            return Err(f"Given reference {self._reference} is invalid")
        ref_obj = self._container_map[self._reference]
        if not isinstance(ref_obj, TypeContainerMixin):
            return Err("Reference doesn't implement TypeMixin")
        match ref_obj.get_value_field():
            case Err(None):
                return Err("Target didn't yield a value field")
            case Ok(c):
                self._inner_type = c
        self._inner_type.allow_dynamic = self.allow_dynamic
        self._inner_type.allow_static = self.allow_static
        self._inner_type.is_static = self.is_static
        return Ok(None)

    @typechecked
    def set_type_map(self, new_map: dict[str, DataContainer[Any]]) -> Result[None, str]:
        """
        Sets the map of data types that the given reference points to.
        This is necessary for reference types to work properly.
        """
        self._container_map = new_map
        return self._set_inner_type()

    @classmethod
    @typechecked
    def _dict_from_msg(cls, msg: NodeDataType) -> Result[dict, str]:
        match super()._dict_from_msg(msg):
            case Err(e):
                return Err(e)
            case Ok(c):
                config = c
        # Reference types don't accept a value param on init
        config.pop("value")
        config["reference"] = msg.reference_target
        return Ok(config)

    def is_compatible(
        self, other: DataContainer
    ) -> TypeGuard["ReferenceContainer"]:  # -> TypeGuard[Self]
        """
        Compatibility here is only evaluated within exact matching references.
        See also :obj:`self.get_runtime_type`
        """
        if not super().is_compatible(other):
            return False
        return self._reference == other._reference

    def serialize_type(self) -> NodeDataType:
        if self._inner_type is None:
            type_msg = super().serialize_type()
        else:
            type_msg = self._inner_type.serialize_type()
            type_msg.type_identifier = self.type_identifier
        type_msg.reference_target = self._reference
        return type_msg

    @typechecked
    def get_runtime_type(self) -> DataContainer:
        """
        Reference types are fully transparent at runtime,
        simply forwarding the type information from the concrete inner type.
        """
        if self._inner_type is None:
            return self
        return self._inner_type.get_runtime_type()

    def has_value(self) -> bool:
        if self._inner_type is None:
            return False
        return self._inner_type.has_value()

    def set_value(self, value: Any) -> Result[None, str]:
        if self._inner_type is None:
            return Err("Can't set value on reference without target")
        return self._inner_type.set_value(value)

    def get_value(self) -> Result[Any, None]:
        if self._inner_type is None:
            return Err(None)
        return self._inner_type.get_value()

    def reset_value(self):
        if self._inner_type is None:
            return None
        return self._inner_type.reset_value()

    def is_updated(self) -> bool:
        if self._inner_type is None:
            return False
        return self._inner_type.is_updated()

    def flag_updated(self) -> None:
        if self._inner_type is None:
            return None
        return self._inner_type.flag_updated()

    def reset_updated(self) -> None:
        if self._inner_type is None:
            return None
        return self._inner_type.reset_updated()

    def _serialize_value(self, value: Any) -> Any:
        if self._inner_type is None:
            return None
        return self._inner_type._serialize_value(value)

    def _deserialize_value(self, ser_value: Any) -> Result[Any, str]:
        if self._inner_type is None:
            return Err("Reference without inner type doesn't accept values")
        return self._inner_type._deserialize_value(ser_value)


@register_io_type
class ReferenceType(ReferenceContainer):
    """
    The basic reference type.
    """

    type_identifier = NodeDataType.REFERENCE_TYPE


class IterableReferenceContainer(ReferenceContainer):
    """
    A container for iterable references.

    Works similar to the standard :obj:`IterableContainer`,
    with the difference that the element type is determined by the referenced type value.
    """

    max_length: int = 2**64 - 1
    strict_length: bool = False

    @typechecked
    def __init__(
        self,
        reference: str,
        max_length: Optional[int] = None,
        strict_length: Optional[bool] = None,
        allow_dynamic: bool = True,
        allow_static: bool = False,
        is_static: Optional[bool] = None,
    ) -> None:
        if max_length is not None:
            self.max_length = max_length
        if strict_length is not None:
            self.strict_length = strict_length

        super().__init__(
            reference=reference,
            allow_dynamic=allow_dynamic,
            allow_static=allow_static,
            is_static=is_static,
        )


@register_io_type
class ReferenceListType(ReferenceContainer):
    """
    A reference type for lists.
    Works like the basic :obj:`ListType`, with the element type being set by reference
    """

    type_identifier = NodeDataType.REFERENCE_LIST_TYPE

    @typechecked
    def _set_inner_type(self) -> Result[None, str]:
        match super()._set_inner_type():
            case Err(e):
                return Err(e)
            case Ok(None):
                pass
        self._inner_type = ListType(element_type=self._inner_type)
        self._inner_type.allow_dynamic = self.allow_dynamic
        self._inner_type.allow_static = self.allow_static
        self._inner_type.is_static = self.is_static
        return Ok(None)


@register_io_type
class ReferenceDictType(ReferenceContainer):
    """
    A reference type for dicts.
    Works like the basic :obj:`DictType`, with the element type being set by reference
    """

    type_identifier = NodeDataType.REFERENCE_DICT_TYPE

    @typechecked
    def _set_inner_type(self) -> Result[None, str]:
        match super()._set_inner_type():
            case Err(e):
                return Err(e)
            case Ok(None):
                pass
        self._inner_type = DictType(element_type=self._inner_type)
        self._inner_type.allow_dynamic = self.allow_dynamic
        self._inner_type.allow_static = self.allow_static
        self._inner_type.is_static = self.is_static
        return Ok(None)


@typechecked
def get_message_field_io_type(field_type: str) -> Result[DataContainer, str]:
    """
    Constructs a data type based on the field type of a ROS message field,
    as given by the :obj:`get_fields_and_field_types()` function.
    """
    # Checks if the type matches a sequence definition and extracts the element-type and bounds
    match_sequence = re.match(r"sequence<([\w\/<>]+)(?:, (\d+))?>", field_type)
    # Checks if the type matches an array definition and extracts the element-type and bounds
    match_array = re.match(r"([\w\/<>]*)\[(\d+)\]", field_type)
    if match_sequence or match_array:
        if match_sequence:
            nested_type_str, max_len_str = match_sequence.groups()
            is_static = False
        if match_array:
            nested_type_str, max_len_str = match_array.groups()
            is_static = True

        if max_len_str is None:
            max_len = None
        else:
            max_len = int(max_len_str)

        match get_message_field_io_type(nested_type_str):
            case Err(e):
                return Err(e)
            case Ok(t):
                element_type = t
        return Ok(
            ListType(
                element_type=element_type, max_length=max_len, strict_length=is_static
            )
        )

    match_string = re.match(r"w?string(?:<(\d+)>)?", field_type)
    if match_string:
        max_len_str = match_string.group(1)
        if max_len_str is None:
            max_len = None
        else:
            max_len = int(max_len_str)
        return Ok(StringType(max_length=max_len))

    # Check if the type matches a message type
    if field_type.find("/") != -1:
        return Ok(
            RosMessageType(
                message_type=rosidl_runtime_py.utilities.get_message(field_type)
            )
        )

    if field_type == "boolean":
        return Ok(BoolType())
    if field_type == "octet":
        return Ok(BytesType())
    if field_type.find("int") != -1:
        try:
            min_v, max_v = INT_LIMITS[field_type]
            return Ok(IntType(min_value=min_v, max_value=max_v))
        except KeyError:
            return Err(f"{field_type} is not an integer type")
    if field_type.find("float") != -1 or field_type.find("double") != -1:
        try:
            min_v, max_v = FLOAT_LIMITS[field_type]
            return Ok(FloatType(min_value=min_v, max_value=max_v))
        except KeyError:
            return Err(f"{field_type} is not a float type")

    return Err(f"Unrecognized field type {field_type}")
