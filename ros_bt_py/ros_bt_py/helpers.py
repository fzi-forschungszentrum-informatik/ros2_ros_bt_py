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

# from enum import StrEnum Not available in Python3.10
import abc
import functools

from typing import cast
from typeguard import typechecked

from ros_bt_py_interfaces.msg import NodeState


# TODO Update this once StrEnum is available in all relevant versions (aka Humble is EOL)
@typechecked
# class BTNodeState(StrEnum):
class BTNodeState(abc.ABC):
    # Lots of duct tape to make type checkers happy.
    #   All of this casting and registering becomes unnecessary
    #   once we can switch to StrEnum, which behaves as expected
    UNINITIALIZED = cast("BTNodeState", NodeState.UNINITIALIZED)
    IDLE = cast("BTNodeState", NodeState.IDLE)
    UNASSIGNED = cast("BTNodeState", NodeState.UNASSIGNED)
    ASSIGNED = cast("BTNodeState", NodeState.ASSIGNED)
    RUNNING = cast("BTNodeState", NodeState.RUNNING)
    SUCCEEDED = cast("BTNodeState", NodeState.SUCCEEDED)
    FAILED = cast("BTNodeState", NodeState.FAILED)
    BROKEN = cast("BTNodeState", NodeState.BROKEN)
    PAUSED = cast("BTNodeState", NodeState.PAUSED)
    SHUTDOWN = cast("BTNodeState", NodeState.SHUTDOWN)
    # These casts are for the purpose of mypy/pylance type checking,
    #   which ignore abc `register` subclasses


BTNodeState.register(str)
# Register `str` as subclass to `BTNodeState` for the purpose of typeguard


INT_LIMITS = {
    "int8": (-(2**7), 2**7 - 1),
    "int16": (-(2**15), 2**15 - 1),
    "int32": (-(2**31), 2**31 - 1),
    "int64": (-(2**63), 2**63 - 1),
    "uint8": (0, 2**8 - 1),
    "uint16": (0, 2**16 - 1),
    "uint32": (0, 2**32 - 1),
    "uint64": (0, 2**64 - 1),
}


def int_limits_dict(name: str) -> dict[str, int]:
    limits = INT_LIMITS[name]
    return {"min_value": limits[0], "max_value": limits[1]}


FLOAT_LIMITS = {
    "float": (-3.4028235e38, 3.4028235e38),
    "double": (-1.7976931348623157e308, 1.7976931348623157e308),
}


def float_limits_dict(name: str) -> dict[str, float]:
    limits = FLOAT_LIMITS[name]
    return {"min_value": limits[0], "max_value": limits[1]}


# Max uint64 value that exactly matches a float64 value
# It is used to prevent precision loss between integers and js numbers
INT_FLOAT_MAX = 2**64 - 1616


# handling nested objects,
# see https://stackoverflow.com/questions/31174295/getattr-and-setattr-on-nested-objects
def rgetattr(obj, attr, *args):
    def _getattr(obj, attr):
        return getattr(obj, attr, *args)

    return functools.reduce(_getattr, [obj] + attr.split("."))


def rsetattr(obj, attr: str, val):
    pre, _, post = attr.rpartition(".")
    return setattr(rgetattr(obj, pre) if pre else obj, post, val)
