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

from unittest.mock import Mock

from ros_bt_py.helpers import BTNodeState
from ros_bt_py.nodes.fallback import MemoryFallback
from ros_bt_py.vendor.result import Ok


def test_untick_preserves_running_child():
    first_child = Mock()
    first_child.tick.return_value = Ok(BTNodeState.FAILED)
    first_child.untick.return_value = Ok(BTNodeState.IDLE)

    running_child = Mock()
    running_child.tick.return_value = Ok(BTNodeState.RUNNING)
    running_child.untick.return_value = Ok(BTNodeState.IDLE)

    fallback = MemoryFallback()
    fallback.children = [first_child, running_child]
    fallback.state = BTNodeState.IDLE
    fallback.last_running_child = 0

    assert fallback._do_tick().unwrap() == BTNodeState.RUNNING
    assert fallback.last_running_child == 1

    assert fallback._do_untick().unwrap() == BTNodeState.IDLE
    assert fallback.last_running_child == 1

    assert fallback._do_tick().unwrap() == BTNodeState.RUNNING
    first_child.tick.assert_called_once()
    assert running_child.tick.call_count == 2
