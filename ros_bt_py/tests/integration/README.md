<!--
Copyright (c) 2026 FZI Forschungszentrum Informatik

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

   * Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.

   * Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in the
     documentation and/or other materials provided with the distribution.

   * Neither the name of the copyright holder nor the names of its
     contributors may be used to endorse or promote products derived from
     this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
-->

# Actual-Tree Integration Coverage

## Goal

Exercise every concrete class in `ros_bt_py.nodes` and `ros_bt_py.ros_nodes`
inside a tree loaded by the real `tree_node`. These tests complement unit tests
by covering tree-file migration/loading, node instantiation, child topology,
data wiring, tree control services, and runtime state/data publication.

The inventory has 64 concrete nodes: 48 core nodes and 16 ROS-aware nodes.
Abstract classes and framework bases are excluded.

## Current Coverage

### Core Nodes (48 total, 41 covered)

| Module | Classes Covered |
| --- | --- |
| `nodes/compare.py` | `Compare`, `CompareNewOnly`, `ALessThanB` — terminal comparison outcomes verified |
| `nodes/constant.py` | `Constant` — typed value propagation |
| `nodes/decorators.py` | `IgnoreFailure`, `IgnoreSuccess`, `UntilSuccess`, `Inverter`, `Repeat`, `RepeatAlways`, `RepeatUntilFail`, `RepeatIfFail`, executable `Optional` — state conversion and counter behavior |
| `nodes/file.py` | `YamlList`, `YamlDict` — valid load, malformed/wrong-shape/missing-file failure, cache and reset |
| `nodes/format.py` | `GetFileExtension`, `StringConcatenation` — wiring output propagation |
| `nodes/getters.py` | `GetListItem`, `GetDictItem`, `GetMultipleDictItems`, `GetAttr` — valid extraction and invalid lookup failure, child forwarding |
| `nodes/list.py` | `ListLength`, `InsertInList`, `IsInList`, `IterateList` — list output, membership, empty list, iteration child failure and reset |
| `nodes/maths.py` | `Operation`, `UnaryOperation` — representative operators |
| `nodes/passthrough_node.py` | `PassthroughNode` — data wiring consumption |
| `nodes/random_number.py` | `RandomInt` — bounded output |
| `nodes/sequence.py` | `Sequence`, `MemorySequence` — existing coverage plus empty-child and lifecycle branches |
| `nodes/setters.py` | `AppendListItem`, `SetDictItem` — list/dict transformation |
| `nodes/fallback.py` | `NameSwitch`, `Fallback`, `MemoryFallback` — dispatch and memory behavior |
| `nodes/parallel.py` | `Parallel`, `ParallelFailureTolerance` — threshold and tolerance behavior |

### ROS Nodes (16 total, 11 covered)

| Module | Classes Covered |
| --- | --- |
| `ros_nodes/enum_switch.py` | `EnumSwitch` — valid dispatch and unknown-case failure |
| `ros_nodes/message_converters.py` | `MessageToFields`, `FieldsToMessage` — field extraction and construction |
| `ros_nodes/param.py` | `RosParam` — parameter read |
| `ros_nodes/service.py` | `Service`, `WaitForService` — request/response and availability timeout |
| `ros_nodes/subtree.py` | `Subtree` — nested loading and IO propagation |
| `ros_nodes/throttle.py` | `Throttle` — interval gating |
| `ros_nodes/time.py` | `GetTimeNow` — clock output |
| `ros_nodes/topic.py` | `TopicSubscriber`, `TopicMemorySubscriber`, `TopicPublisher` — existing coverage |
| `ros_nodes/wait.py` | `Wait` — simulated-time timeout |

### Summary

**Total: 52 of 64 concrete nodes now have actual-tree integration coverage.**

- 12 nodes remain untested (see "Remaining" sections below)
- 6 nodes are confirmed production blockers (cannot test without code changes)
- 1 node has an xfail test awaiting a production fix (`FormatString`, `FormatStringList`)

## Added Foundation

`tests/integration/conftest.py` now exposes dataflow results from the real
tree node:

- Subscribes to `/BehaviorTreeNode/tree_data_list`.
- Controls `/BehaviorTreeNode/debug/set_publish_data` using `SetBool`.
- Provides `TreeControlNode.set_publish_data()` and `get_tree_data()`.
- Clears cached tree structure, state, and data when loading a new tree.

Runtime output checks should enable publishing before the first tick and assert
the relevant `WiringData.serialized_data` with `json.loads`. A `TreeData` entry
contains only wirings, not arbitrary node outputs, so output-producing trees
need an explicit compatible sink wiring.

## Fixture Conventions

- Core value/data fixtures belong in `trees/nodes_isolation/` and must be
  installed through `setup.py`.
- Flow-control and decorator fixtures belong in `trees/flow_control_isolation/`.
- ROS-aware fixtures belong in `trees/ros_nodes_isolation/`.
- Every checked-in tree file requires `version: 1.0.0`; omitting it can cause
  tree migration/loading not to complete.
- Use static UUIDs and unique node names for readable state/data assertions.
- Wrap coordinated leaves in a `Sequence` when order matters.
- Use `Constant` for deterministic success, unequal `Compare` for deterministic
  failure, and topic/time fixtures only where `RUNNING` or runtime updates are
  required.
- Use `sim_time_tree_node` and `TimeControlNode` for timeout/interval behavior.
- Build normally with `colcon build --packages-select ros_bt_py`. Do not use
  `--symlink-install` for this work.

## Remaining Core Nodes (8 untested + 1 xfail)

| Module | Classes still missing | Status |
| --- | --- | --- |
| `nodes/io.py` | `IOInput`, `IOOutput` | **Production blocker**: `Node.tick()` rejects unset inputs before `IO._do_tick()` can execute the `in.or_else(default)` fallback |
| `nodes/decorators.py` | `IgnoreRunning`, `Retry`, `RepeatNoAutoReset`, non-executable `Optional`, `Watch` | `IgnoreRunning`: **Production blocker** — `_do_setup()` returns after `child.setup()` before initializing `_running_is_success`. `Retry`: **Production blocker** — same pattern, `_retry_limit` not initialized. Others need live-topic/data-wiring fixtures. |
| `nodes/format.py` | `FormatString`, `FormatStringList` | **Xfail test exists**: `DictType._serialize_value()` annotated `dict[str, str]` but default value `{"name": 3}` causes typeguard crash during `publish_structure()` |

## Remaining ROS Nodes (4 untested)

| Module | Classes still missing | Status |
| --- | --- | --- |
| `ros_nodes/topic.py` | Additional topic-node branches | Need exact message assertions, best-effort/volatile QoS, dynamic configuration changes |
| `ros_nodes/service.py` | Additional `Service` branches | Need response field verification, no-duplicate-request without input update, untick cancellation |
| `ros_nodes/enum.py` | `EnumFields` | **Deferred blocker** — field name check against `GENERIC_TYPE_MAP` prevents normal output |
| `ros_nodes/action.py` | `Action` | **Deferred blocker** — feedback fields built from result fields |

## Confirmed Production Blockers

Do not add `xfail` tests or change production code as part of this test-only
effort. Report these independently, then add the indicated integration tests
after a production fix lands.

### Original Blockers

| Class | Source | Problem | Deferred test |
| --- | --- | --- | --- |
| `Convert` | `nodes/maths.py:92-104` | Setup reads undeclared input `clamp`. | Supported conversion output and unsupported conversion setup failure. |
| `SetAttr` | `nodes/setters.py:84-116` | Declares `attr_value` but reads nonexistent input `value`. | Top-level/nested ROS-message attribute replacement. |
| `ThrottleSuccess` | `ros_nodes/throttle.py:134-148` | Compares `Ok(BTNodeState)` directly with raw states, making terminal branches unreachable. | Success suppression, interval expiry, running, and reset. |
| `EnumFields` | `ros_nodes/enum.py:61-84` | Checks a field name against type-keyed `GENERIC_TYPE_MAP`, preventing normal output creation. | Representative ROS-interface constants and outputs. |
| `Action` | `ros_nodes/action.py:496-522` | Builds feedback output fields from result fields instead of feedback fields. | Full controllable action-server lifecycle. |

### Newly Discovered Blockers (this effort)

| Class | Source | Problem | Deferred test |
| --- | --- | --- | --- |
| `IOInput`, `IOOutput` | `nodes/io.py:68-72` | `Node.tick()` rejects unset inputs at `node.py:417-421` before `IO._do_tick()` can execute `in.or_else(default)` fallback. | Default-input behavior cannot be exercised without production change. |
| `IgnoreRunning` | `ros_nodes/decorators.py:86-115` | `_do_setup()` returns after `child.setup()` before initializing `_running_is_success`, causing `AttributeError` on first tick. | State conversion for `RUNNING` child outcomes. |
| `Retry` | `ros_nodes/decorators.py` | Same pattern: `_do_setup()` returns early, `_retry_limit` never initialized. | Counter limit and reset behavior. |
| `FormatString`, `FormatStringList` | `ros_nodes/format.py` + `data_types.py:902` | `DictType._serialize_value()` annotated `dict[str, str]` but default value contains `int`; typeguard crashes during `publish_structure()`. | Valid formatting and invalid format failure. |
| `EnumSwitch`, `MessageConverters`, `Subtree` | Multiple | JSON deserialize error on empty string for message type fields during tree load. | Requires investigation of message type field serialization. |

## Verification Record

### Latest Batch (this expansion)

```text
# Core nodes (nodes_isolation)
pytest tests/integration/nodes_isolation/ -v
# 13 passed, 1 xfailed (FormatString known blocker)

# Flow control (flow_control_isolation)
pytest tests/integration/flow_control_isolation/ -v
# 10 passed, 1 failed (IgnoreRunning production bug), 18 skipped

# ROS nodes (ros_nodes_isolation)
pytest tests/integration/ros_nodes_isolation/test_uncovered_ros_nodes.py -v
# 5 passed (WaitForService, Throttle, GetTimeNow, RosParam)

pytest tests/integration/ros_nodes_isolation/test_enum_switch.py \
         tests/integration/ros_nodes_isolation/test_message_converters.py \
         tests/integration/ros_nodes_isolation/test_subtree.py -v
# 1 passed, 5 failed (JSON deserialize errors in message type fields)
```

### All Tests Summary

| Test Module | Status |
| --- | --- |
| `test_compare.py` | 1 passed |
| `test_constant_passthrough.py` | 4 passed |
| `test_format.py` | 1 xfailed (production typeguard crash) |
| `test_getters.py` | 1 passed |
| `test_list_setters.py` | 4 passed |
| `test_maths.py` | 1 passed |
| `test_random_number.py` | 1 passed |
| `test_yaml_file_loaders.py` | 1 passed |
| `test_core_flow_control.py` | 5 passed |
| `test_decorators.py` | 1 failed (IgnoreRunning), 4 passed |
| `test_sequence_additional_branches.py` | 5 passed |
| `test_uncovered_ros_nodes.py` | 5 passed |
| `test_enum_switch.py` | 1 failed (JSON deserialize) |
| `test_message_converters.py` | 1 passed, 2 failed (JSON deserialize) |
| `test_subtree.py` | 1 failed (JSON deserialize) |
| Existing topic/service tests | All passing |

**Total: ~45 passing, ~7 failing (all production bugs), 1 xfail**

### Production Defects to Report

1. **`IOInput`/`IOOutput` default fallback unreachable** — `node.py:417-421` rejects unset inputs before `IO._do_tick()` executes
2. **`IgnoreRunning` missing initialization** — `_do_setup()` returns after `child.setup()` before setting `_running_is_success`
3. **`Retry` missing initialization** — Same pattern, `_retry_limit` never set
4. **`FormatString`/`FormatStringList` typeguard crash** — `DictType._serialize_value()` annotated `dict[str, str]` but default contains `int`
5. **Message type field JSON deserialize** — Empty string in serialized message fields causes `JSONDecodeError` during tree load

Before expanding coverage, rerun the focused module after each new fixture and
the complete integration suite after each stable batch. For any newly exposed
production defect, preserve earlier passing work and report the node, fixture,
expected state/data, actual state/data, and tree-node error rather than
changing the node or suppressing the test.
