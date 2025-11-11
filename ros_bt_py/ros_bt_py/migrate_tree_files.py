# Copyright 2025 FZI Forschungszentrum Informatik
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
import importlib
import itertools
import uuid
import os
import sys
import yaml
from importlib import metadata
from packaging.version import Version
from typing import Literal
from result import Result, Ok, Err
from ros_bt_py.node import Node
from ros_bt_py.custom_types import FilePath, RosActionName, RosActionType, RosServiceName, RosServiceType, RosTopicName, RosTopicType, TypeWrapper
from ros_bt_py.helpers import json_decode, json_encode
from ros_bt_py.node_config import OptionRef
from ros_bt_py.ros_helpers import get_interface_name

def filter_node_candidates(
    nodes: list[type[Node]],
    key: Literal['options', 'inputs', 'outputs'],
    values: set[str],
) -> list[type[Node]]:
    equal = []
    extra = []
    missing = []
    for candidate in nodes:
        if candidate._node_config is None:
            continue
        class_keys = getattr(candidate._node_config, key, {}).keys()
        if class_keys == values:
            equal.append(candidate)
        if class_keys < values:
            extra.append(candidate)
        if class_keys > values:
            missing.append(candidate)
    if len(equal) > 0:
        return equal
    if len(extra) > 0:
        return extra
    return missing

def find_node_class(
    module: str,
    class_name: str,
    options: set[str],
    inputs: set[str],
    outputs: set[str],
) -> Result[type[Node], str]:
    if module not in Node.node_classes.keys():
        _ = importlib.import_module(module)
    if class_name not in Node.node_classes[module].keys():
        return Err(f"Could not find class `{module}.{class_name}`.")

    class_list = Node.node_classes[module][class_name]
    if len(class_list) == 1:
        return Ok(class_list[0])

    option_candidates = filter_node_candidates(class_list, 'options', options)
    if len(option_candidates) == 1:
        return Ok(option_candidates[0])

    input_candidates = filter_node_candidates(option_candidates, 'inputs', inputs)
    if len(input_candidates) == 1:
        return Ok(input_candidates[0])

    output_candidates = filter_node_candidates(input_candidates, 'outputs', outputs)
    if len(output_candidates) >= 1:
        return Ok(output_candidates[0])

    return Err(f"Could not find class `{module}.{class_name}`.")


def update_node_option(option_dict: dict, option_type: type) -> Result[dict, str]:
    # Early out if types are matching
    if json_encode(option_type) == option_dict['serialized_type']:
        return Ok(option_dict)

    # Leave optionrefs untouched
    if isinstance(option_type, OptionRef):
        return Ok(option_dict)

    if isinstance(option_type, TypeWrapper):
        match update_node_option(
            option_dict,
            option_type.actual_type
        ):
            case Err(e):
                return Err(e)
            case Ok(opt_dict):
                opt_dict['serialized_type'] = json_encode(option_type)
                return Ok(opt_dict)

    if option_type in [
        RosServiceName,
        RosTopicName,
        RosActionName,
        FilePath,
    ]:
        option_dict['serialized_type'] = json_encode(option_type)
        option_dict['serialized_value'] = json_encode(
            option_type(json_decode(option_dict['serialized_value']))
        )
        return Ok(option_dict)

    if option_type in [
        RosServiceType,
        RosActionType,
        RosTopicType,
    ]:
        option_dict['serialized_type'] = json_encode(option_type)
        option_dict['serialized_value'] = json_encode(
            option_type(get_interface_name(
                json_decode(option_dict['serialized_value'])  # type: ignore
            ))
        )
        return Ok(option_dict)

    return Err(f"Cannot convert option config {option_dict} to type {option_type}")


def update_node_configs(node_dict: dict) -> Result[dict, str]:
    node_dict.pop('option_wirings', None)
    node_dict.pop('state', None)
    match find_node_class(
        node_dict['module'],
        node_dict['node_class'],
        { opt['key'] for opt in node_dict['options'] },
        { inp['key'] for inp in node_dict['inputs'] },
        { outp['key'] for outp in node_dict['outputs'] },
    ):
        case Err(e):
            return Err(e)
        case Ok(c):
            node_class = c
    if node_class._node_config is None:
        return Err(f"Node class `{node_class}` cannot be initialized.")
    new_node_options = []
    for node_option in node_dict['options']:
        if node_option['key'] not in node_class._node_config.options:
            continue
        match update_node_option(
            node_option,
            node_class._node_config.options[node_option['key']]
        ):
            case Err(e):
                return Err(e)
            case Ok(opt):
                new_node_options.append(opt)
    node_dict['options'] = new_node_options
    for node_io in itertools.chain(node_dict['inputs'], node_dict['outputs']):
        _ = node_io.pop('serialized_value', None)
    return Ok(node_dict)


def assign_uuids(tree_dict: dict) -> dict:
    """This assumes a legacy structure that uses names to identify nodes"""
    mapping: dict[str, uuid.UUID] = {}
    for node_dict in tree_dict['nodes']:
        node_id = uuid.uuid4()
        mapping[node_dict['name']] = node_id
        node_dict['node_id'] = str(node_id)

    for node_dict in tree_dict['nodes']:
        node_dict['child_ids'] = [
            str(mapping[name])
            for name in node_dict.pop('child_names')
        ]
    for wiring in tree_dict['data_wirings']:
        for point in ['source', 'target']:
            wiring[point]['node_id'] = str(
                mapping[
                    wiring[point].pop('node_name')
                ]
            )
    for public_data in tree_dict['public_node_data']:
        public_data['node_id'] = str(
            mapping[
                public_data.pop('node_name')
            ]
        )

    tree_dict['tree_id'] = str(uuid.UUID(int=0))
    tree_dict['root_id'] = str(mapping[tree_dict.pop('root_name')])
    return tree_dict


def migrate_legacy_tree_structure(tree_dict: dict) -> Result[dict, str]:
    # Since the option migrations are safe (and could be extended later),
    #   we always run them if the version is not up-to-date.
    tree_version = Version(tree_dict.get('version', '0.0.0'))
    if tree_version < Version(metadata.version('ros_bt_py')):
        tree_dict.pop('state', None)
        new_nodes = []
        for node_dict in tree_dict['nodes']:
            match update_node_configs(node_dict):
                case Err(e):
                    return Err(e)
                case Ok(new_dict):
                    new_nodes.append(new_dict)
        tree_dict['nodes'] = new_nodes

    # If the version is older than 0.6.0, we have to introduce uuids.
    if tree_version < Version('0.6.0'):
        tree_dict = assign_uuids(tree_dict)

    # Once all migrations are successful, update the version number
    tree_dict['version'] = metadata.version('ros_bt_py')
    return Ok(tree_dict)


def main():
    for file in sys.argv[1:]:
        print(f"Migrating file {file}")

        name, ext = os.path.splitext(file)
        new_file = f"{name}_new{ext}"
        counter = 1
        while os.path.exists(new_file):
            new_file = f"{name}_new_{counter}{ext}"
            counter += 1

        with open(file, 'r') as f:
            tree_dict = yaml.safe_load(f.read())

        match migrate_legacy_tree_structure(tree_dict):
            case Err(e):
                print(f"Failed to migrate: {e}")
                continue
            case Ok(d):
                new_tree_dict = d

        with open(new_file, 'w+') as f:
            f.write(yaml.safe_dump(new_tree_dict, sort_keys=False))
        print(f"Saved migration to {new_file}")



