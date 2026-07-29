# Copyright 2026 FZI Forschungszentrum Informatik
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
from typing import TYPE_CHECKING, NamedTuple
import uuid

from ros_bt_py.vendor.result import Result, Ok, Err

from ros_bt_py.data_types import DataContainer
from ros_bt_py.ros_helpers import ros_to_uuid, uuid_to_ros

if TYPE_CHECKING:
    from ros_bt_py.node import Node

from ros_bt_py_interfaces.msg import Wiring, WiringData


Connection = NamedTuple(
    "Connection",
    [
        (
            "source_key",
            str,
        ),
        (
            "target_id",
            uuid.UUID,
        ),
        (
            "target_key",
            str,
        ),
    ],
)


class DataFlowManager:
    """
    This manages the dataflow for a set of nodes and wirings (usually a tree instance).

    On construction we can pass a reference to dictionaries of incoming and outgoing data.
    The keys of those dictionaries are expected to be in the format `node_id.data_key`.
    Keys that cannot be matched are silently ignored, so passing extra is safe.
    """

    nodes: dict[uuid.UUID, "Node"]
    connections: dict[uuid.UUID, list[Connection]]
    incoming_data: dict[str, DataContainer]
    outgoing_data: dict[str, DataContainer]

    def __init__(
        self,
        incoming_data: dict[str, DataContainer] = {},
        outgoing_data: dict[str, DataContainer] = {},
    ) -> None:
        self.incoming_data = incoming_data
        self.outgoing_data = outgoing_data

    def initialize(
        self,
        nodes: dict[uuid.UUID, "Node"],
        wirings: list[Wiring],
    ) -> Result[None, str]:
        self.nodes = nodes
        self.connections = {node_id: [] for node_id in self.nodes.keys()}
        for wiring in wirings:
            match ros_to_uuid(wiring.source.node_id):
                case Err(e):
                    return Err(e)
                case Ok(u):
                    source_id = u
            source_key = wiring.source.data_key
            match ros_to_uuid(wiring.target.node_id):
                case Err(e):
                    return Err(e)
                case Ok(u):
                    target_id = u
            target_key = wiring.target.data_key
            self.connections[source_id].append(
                Connection(
                    source_key=source_key,
                    target_id=target_id,
                    target_key=target_key,
                )
            )
        return Ok(None)

    def push_outputs(self, node_id: uuid.UUID) -> Result[None, str]:
        try:
            source_node = self.nodes[node_id]
            connections = self.connections[node_id]
        except KeyError:
            return Err(f"Source node id {node_id} is not available")
        for connection in connections:
            try:
                if not source_node.node_config.outputs[
                    connection.source_key
                ].is_updated():
                    continue
                match source_node.node_config.outputs[
                    connection.source_key
                ].get_value():
                    case Err(None):
                        continue
                    case Ok(v):
                        value = v
                target_node = self.nodes[connection.target_id]
                match target_node.node_config.inputs[connection.target_key].set_value(
                    value
                ):
                    case Err(e):
                        return Err(e)
                    case Ok(None):
                        pass
            except KeyError:
                return Err(f"Connection {connection} is invalid")
        for key, container in source_node.node_config.outputs.items():
            if not container.is_updated():
                continue
            outgoing_key = f"{node_id}.{key}"
            if outgoing_key not in self.outgoing_data.keys():
                continue
            match container.get_value():
                case Err(None):
                    continue
                case Ok(v):
                    value = v
            match self.outgoing_data[outgoing_key].set_value(value):
                case Err(e):
                    return Err(e)
                case Ok(None):
                    pass
        return Ok(None)

    def push_incoming_data(self) -> Result[None, str]:
        for incoming_key, container in self.incoming_data.items():
            if not container.is_updated():
                continue
            match container.get_value():
                case Err(None):
                    continue
                case Ok(v):
                    value = v
            try:
                node_id, data_key = incoming_key.split(".")
                node_id = uuid.UUID(node_id)
            except ValueError:
                continue
            try:
                target_node = self.nodes[node_id]
                match target_node.node_config.inputs[data_key].set_value(value):
                    case Err(e):
                        return Err(e)
                    case Ok(None):
                        pass
            except KeyError:
                continue
        return Ok(None)

    def _get_wiring_data(
        self,
        source_id: uuid.UUID,
        source_key: str,
        target_id: uuid.UUID,
        target_key: str,
    ) -> WiringData:
        wiring_data = WiringData()
        wiring_data.wiring.source.node_id = uuid_to_ros(source_id)
        wiring_data.wiring.source.data_key = source_key
        wiring_data.wiring.target.node_id = uuid_to_ros(target_id)
        wiring_data.wiring.target.data_key = target_key
        try:
            source_container = self.nodes[source_id].node_config.outputs[source_key]
            target_container = self.nodes[target_id].node_config.inputs[target_key]
            wiring_data.source_type = (
                source_container.get_runtime_type().serialize_type()
            )
            wiring_data.target_type = (
                target_container.get_runtime_type().serialize_type()
            )
            wiring_data.serialized_data = source_container.serialize_value()
            wiring_data.is_current = (
                wiring_data.serialized_data == target_container.serialize_value()
            )
        except KeyError:
            pass
        return wiring_data

    def get_wiring_data(self) -> list[WiringData]:
        wiring_list = []
        for source_id, connections in self.connections.items():
            for connection in connections:
                wiring_list.append(
                    self._get_wiring_data(
                        source_id,
                        connection.source_key,
                        connection.target_id,
                        connection.target_key,
                    )
                )
        return wiring_list
