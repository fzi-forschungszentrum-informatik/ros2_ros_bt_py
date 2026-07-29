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

from ros_bt_py.vendor.result import Ok, Err

from ros_bt_py.data_flow_manager import DataFlowManager
from ros_bt_py.data_types import IntType, GenericType
from ros_bt_py.helpers import BTNodeState

from ros_bt_py.nodes.compare import Compare
from ros_bt_py.nodes.constant import Constant
from ros_bt_py.nodes.passthrough_node import PassthroughNode

from ros_bt_py_interfaces.msg import Wiring, WiringData


def test_passthrough():
    node_id = uuid.UUID(int=1)

    incoming_data = {f"{node_id}.in": IntType(value=42)}
    output_container = IntType(allow_dynamic=False)
    outgoing_data = {f"{node_id}.out": output_container}

    passthrough_node = PassthroughNode(
        node_id=node_id,
        new_inputs={
            "passthrough_type": GenericType(valid_types=[int]),
        },
    )

    data_flow_manager = DataFlowManager(incoming_data, outgoing_data)

    assert data_flow_manager.initialize({node_id: passthrough_node}, []).is_ok()

    assert passthrough_node.setup().is_ok()

    assert data_flow_manager.push_incoming_data().is_ok()

    # Since we did not register the data flow manager with the node, we manually push outputs.
    assert passthrough_node.tick().is_ok()

    assert data_flow_manager.push_outputs(node_id).is_ok()

    match output_container.get_value():
        case Err(e):
            assert False, e
        case Ok(v):
            value = v
    assert value == 42


def test_wiring_data():
    node_id_1 = uuid.UUID(int=1)
    node_id_2 = uuid.UUID(int=2)

    constant_node = Constant(
        node_id=node_id_1, new_inputs={"constant_type": GenericType(valid_types=[int])}
    )
    assert constant_node.node_config.inputs["constant_value"].set_value(42).is_ok()

    compare_node = Compare(
        node_id=node_id_2, new_inputs={"compare_type": GenericType(valid_types=[int])}
    )
    assert compare_node.node_config.inputs["b"].set_value(42).is_ok()

    wiring = Wiring()
    wiring.source.node_id = str(node_id_1)
    wiring.source.data_key = "constant"
    wiring.target.node_id = str(node_id_2)
    wiring.target.data_key = "a"

    data_flow_manager = DataFlowManager()
    assert data_flow_manager.initialize(
        {node_id_1: constant_node, node_id_2: compare_node}, [wiring]
    ).is_ok()

    assert constant_node.setup().is_ok()
    assert compare_node.setup().is_ok()

    assert constant_node.tick().is_ok()

    # Since we did not register the data flow manager with the node, we manually push outputs.
    assert data_flow_manager.push_outputs(node_id_1).is_ok()

    assert compare_node.tick().is_ok()
    assert compare_node.state == BTNodeState.SUCCEEDED

    wiring_list = data_flow_manager.get_wiring_data()
    assert len(wiring_list) == 1

    wiring_data: WiringData = wiring_list[0]
    assert wiring_data.wiring == wiring
    assert wiring_data.serialized_data == "42"
