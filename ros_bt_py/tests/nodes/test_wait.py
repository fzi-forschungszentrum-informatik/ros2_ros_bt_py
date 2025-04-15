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
from typing import Dict
import pytest
import time

from ros_bt_py.nodes.wait import Wait, WaitInput
from ros_bt_py_interfaces.msg import NodeState


class TestWait:
    @pytest.mark.parametrize(
        "wait_time",
        [
            (1.0),
            (0.05),
        ],
    )
    def test_node_success(self, wait_time: float):
        wait_node = Wait(options={"seconds_to_wait": wait_time})
        wait_node.setup()
        assert wait_node.state == NodeState.IDLE
        start_time = time.time()
        while NodeState.RUNNING == wait_node.tick():
            #        assert time.time() - start_time < wait_time
            pass
        end_time = time.time()
        assert NodeState.SUCCEED == wait_node.state
        assert end_time - start_time >= wait_time
        assert end_time - start_time < wait_time * 1.1

    @pytest.mark.parametrize(
        "wait_time",
        [
            (1.0),
            (0.05),
        ],
    )
    def test_node_shutdown(self, wait_time: float):
        wait_node = Wait(options={"seconds_to_wait": wait_time})
        wait_node.setup()
        assert wait_node.state == NodeState.IDLE
        start_time = time.time()
        while NodeState.RUNNING == wait_node.tick():
            #        assert time.time() - start_time < wait_time
            pass
        end_time = time.time()
        assert NodeState.SUCCEED == wait_node.state
        assert end_time - start_time >= wait_time
        assert end_time - start_time < wait_time * 1.1

        wait_node.shutdown()
        assert wait_node.state == NodeState.SHUTDOWN

        wait_node.setup()
        assert wait_node.state == NodeState.IDLE
        start_time = time.time()
        while NodeState.RUNNING == wait_node.tick():
            #        assert time.time() - start_time < wait_time
            pass
        end_time = time.time()
        assert NodeState.SUCCEED == wait_node.state
        assert end_time - start_time >= wait_time
        assert end_time - start_time < wait_time * 1.1

    @pytest.mark.parametrize(
        "wait_time",
        [
            (1.0),
            (0.05),
        ],
    )
    def test_node_reset(self, wait_time: float):
        wait_node = Wait(options={"seconds_to_wait": wait_time})
        wait_node.setup()
        assert wait_node.state == NodeState.IDLE
        start_time = time.time()
        while NodeState.RUNNING == wait_node.tick():
            #        assert time.time() - start_time < wait_time
            pass
        end_time = time.time()
        assert NodeState.SUCCEED == wait_node.state
        assert end_time - start_time >= wait_time
        assert end_time - start_time < wait_time * 1.1

        wait_node.reset()
        assert wait_node.state == NodeState.IDLE

        start_time = time.time()
        while NodeState.RUNNING == wait_node.tick():
            #        assert time.time() - start_time < wait_time
            pass
        end_time = time.time()
        assert NodeState.SUCCEED == wait_node.state
        assert end_time - start_time >= wait_time
        assert end_time - start_time < wait_time * 1.1


class TestWaitInput:
    @pytest.mark.parametrize(
        "wait_time",
        [
            (1.0),
            (0.05),
        ],
    )
    def test_node_success(self, wait_time: float):
        wait_node = WaitInput()
        wait_node.setup()
        assert wait_node.state == NodeState.IDLE

        wait_node.inputs["seconds_to_wait"] = wait_time

        start_time = time.time()
        while NodeState.RUNNING == wait_node.tick():
            #        assert time.time() - start_time < wait_time
            pass
        end_time = time.time()
        assert NodeState.SUCCEED == wait_node.state
        assert end_time - start_time >= wait_time
        assert end_time - start_time < wait_time * 1.1

    @pytest.mark.parametrize(
        "wait_time",
        [
            (1.0),
            (0.05),
        ],
    )
    def test_node_shutdown(self, wait_time: float):
        wait_node = WaitInput()
        wait_node.setup()
        assert wait_node.state == NodeState.IDLE

        wait_node.inputs["seconds_to_wait"] = wait_time

        start_time = time.time()
        while NodeState.RUNNING == wait_node.tick():
            #        assert time.time() - start_time < wait_time
            pass
        end_time = time.time()
        assert NodeState.SUCCEED == wait_node.state
        assert end_time - start_time >= wait_time
        assert end_time - start_time < wait_time * 1.1

        wait_node.shutdown()
        assert wait_node.state == NodeState.SHUTDOWN

        wait_node.setup()
        assert wait_node.state == NodeState.IDLE
        start_time = time.time()
        while NodeState.RUNNING == wait_node.tick():
            #        assert time.time() - start_time < wait_time
            pass
        end_time = time.time()
        assert NodeState.SUCCEED == wait_node.state
        assert end_time - start_time >= wait_time
        assert end_time - start_time < wait_time * 1.1

    @pytest.mark.parametrize(
        "wait_time",
        [
            (1.0),
            (0.05),
        ],
    )
    def test_node_reset(self, wait_time: float):
        wait_node = WaitInput()
        wait_node.setup()
        assert wait_node.state == NodeState.IDLE

        wait_node.inputs["seconds_to_wait"] = wait_time

        start_time = time.time()
        while NodeState.RUNNING == wait_node.tick():
            #        assert time.time() - start_time < wait_time
            pass
        end_time = time.time()
        assert NodeState.SUCCEED == wait_node.state
        assert end_time - start_time >= wait_time
        assert end_time - start_time < wait_time * 1.1

        wait_node.reset()
        assert wait_node.state == NodeState.IDLE

        start_time = time.time()
        while NodeState.RUNNING == wait_node.tick():
            #        assert time.time() - start_time < wait_time
            pass
        end_time = time.time()
        assert NodeState.SUCCEED == wait_node.state
        assert end_time - start_time >= wait_time
        assert end_time - start_time < wait_time * 1.1
