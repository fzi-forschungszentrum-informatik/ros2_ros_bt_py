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
import random

from ros_bt_py_interfaces.msg import Node as NodeMsg

from ros_bt_py.exceptions import BehaviorTreeException

from ros_bt_py.node import Leaf, define_bt_node
from ros_bt_py.node_config import NodeConfig, OptionRef


@define_bt_node(
    NodeConfig(
        version="0.1.0",
        options={"min": int, "max": int},
        inputs={},
        outputs={"random_number": int},
        max_children=0,
    )
)
class RandomInt(Leaf):
    """Provides a pseudo-random integer in range min <= random_number <= max."""

    def _do_setup(self):
        validate_range(self.options["min"], self.options["max"])

    def _do_tick(self):
        validate_range(self.options["min"], self.options["max"])
        self.outputs["random_number"] = random.randrange(
            self.options["min"], self.options["max"] + 1
        )
        return NodeMsg.SUCCEEDED

    def _do_shutdown(self):
        pass

    def _do_reset(self):
        return NodeMsg.IDLE

    def _do_untick(self):
        return NodeMsg.IDLE


@define_bt_node(
    NodeConfig(
        version="0.1.0",
        options={},
        inputs={"min": int, "max": int},
        outputs={"random_number": int},
        max_children=0,
    )
)
class RandomIntInputs(Leaf):
    """Provides a pseudo-random integer in range min <= random_number <= max."""

    def _do_setup(self):
        pass

    def _do_tick(self):
        validate_range(self.inputs["min"], self.inputs["max"])
        self.outputs["random_number"] = random.randrange(
            self.inputs["min"], self.inputs["max"] + 1
        )
        return NodeMsg.SUCCEEDED

    def _do_shutdown(self):
        pass

    def _do_reset(self):
        return NodeMsg.IDLE

    def _do_untick(self):
        return NodeMsg.IDLE


def validate_range(minimum, maximum):
    """Check if `minimum` < `maximum` and raises a BehaviorTreeException if not."""
    if minimum == maximum:
        raise BehaviorTreeException(
            f"minimum ({minimum}) cannot be equal to maximum ({maximum})"
        )
    if minimum > maximum:
        raise BehaviorTreeException(
            f"minimum ({minimum}) cannot be greater that maximum ({maximum})"
        )
