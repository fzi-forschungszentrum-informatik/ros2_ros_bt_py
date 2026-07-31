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

| Class | Location | Coverage |
| --- | --- | --- |
| `Sequence` | `nodes/sequence.py` | Existing launch test covers running, success, failure, auto-reset, and topic-driven child ordering. |
| `MemorySequence` | `nodes/sequence.py` | Existing launch test covers running, success, auto-reset, and memory behavior. |
| `TopicSubscriber` | `ros_nodes/topic.py` | Existing launch test covers message receipt, reset, and transient-local delivery. |
| `TopicMemorySubscriber` | `ros_nodes/topic.py` | Existing launch test covers memory, timeout, reset, and transient-local delivery. |
| `TopicPublisher` | `ros_nodes/topic.py` | Existing launch test covers publish-on-update and reset. |
| `Service` | `ros_nodes/service.py` | Existing launch test covers request, response, and timeout. |
| `Constant` | `nodes/constant.py` | New actual-tree test asserts a typed `int` is propagated over a wiring. |
| `PassthroughNode` | `nodes/passthrough_node.py` | New actual-tree test consumes `Constant` output in the same tree tick. |
| `StringConcatenation` | `nodes/format.py` | New actual-tree test asserts the first concatenation is consumed by a later node. |
| `GetFileExtension` | `nodes/format.py` | New actual-tree test asserts both filename and extension wiring values. |
| `Wait` | `ros_nodes/wait.py` | New simulated-time test covers `RUNNING`, `SUCCEEDED`, and restart after reset. |

The current total is 11 covered classes and 53 classes still to cover.

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

## Remaining Core Nodes

| Module | Classes still missing | Required public behavior |
| --- | --- | --- |
| `nodes/io.py` | `IOInput`, `IOOutput` | Wired input, default-input behavior, and output propagation. Investigate whether the base `Node.tick()` unset-input check prevents the documented default fallback. |
| `nodes/getters.py` | `GetListItem`, `GetDictItem`, `GetMultipleDictItems`, `GetAttr` | Valid extraction, invalid lookup failure, stale-data `RUNNING`, `succeed_on_stale_data`, and child forwarding. |
| `nodes/setters.py` | `AppendListItem`, `SetDictItem` | Exact updated list/dict output. `SetAttr` is blocked below. |
| `nodes/compare.py` | `Compare`, `CompareNewOnly`, `ALessThanB` | Success/failure comparisons and stale-data `RUNNING`. |
| `nodes/maths.py` | `Operation`, `UnaryOperation` | Representative arithmetic/boolean operators and typed output. `Convert` is blocked below. |
| `nodes/list.py` | `ListLength`, `InsertInList`, `IsInList`, `IterateList` | Exact list output, membership success/failure, empty list, per-item iteration, child failure, and reset. |
| `nodes/format.py` | `FormatString`, `FormatStringList` | Valid formatting, extended conversions, and invalid format failure. |
| `nodes/file.py` | `YamlList`, `YamlDict` | Valid load, malformed/wrong-shape/missing-file failure, error output, cache behavior, and reset/reload. |
| `nodes/random_number.py` | `RandomInt` | Success and output inside the inclusive range; invalid range error. |
| `nodes/sequence.py` | Additional `Sequence`, `MemorySequence` branches | Empty-child behavior, explicit untick/reset, and terminal failure for `MemorySequence`. |
| `nodes/fallback.py` | `NameSwitch`, `Fallback`, `MemoryFallback` | Empty/all-failed behavior, success/running short-circuit, memory resumption, invalid selection, and branch change untick. |
| `nodes/parallel.py` | `Parallel`, `ParallelFailureTolerance` | Success/failure thresholds, mixed terminal/running children, tolerance, cleanup, and automatic restart. |
| `nodes/decorators.py` | `IgnoreFailure`, `IgnoreRunning`, `IgnoreSuccess`, `UntilSuccess`, `Inverter`, `Retry`, `Repeat`, `RepeatNoAutoReset`, `RepeatAlways`, `RepeatUntilFail`, `RepeatIfFail`, `Optional`, `Watch` | State conversion, forwarding, counter limits/reset behavior, optional executability, and watch-triggered child restart. |

## Remaining ROS Nodes

| Module | Classes still missing | Required public behavior |
| --- | --- | --- |
| `ros_nodes/topic.py` | Additional topic-node branches | Assert exact messages, best-effort/volatile QoS, and dynamic configuration changes in addition to existing coverage. |
| `ros_nodes/service.py` | `WaitForService`; additional `Service` branches | Service-available success, unavailable running/timeout, reset timing, response fields, no duplicate request without input update, and untick cancellation. |
| `ros_nodes/throttle.py` | `Throttle` | First result, cached terminal result, interval expiry, running child, and reset. `ThrottleSuccess` is blocked below. |
| `ros_nodes/time.py` | `GetTimeNow` | Exact `sec`/`nanosec` output before and after a controlled clock advance. |
| `ros_nodes/param.py` | `RosParam` | Existing parameter output and missing parameter failure using launch parameter overrides. |
| `ros_nodes/enum_switch.py` | `EnumSwitch` | Valid dispatch, missing child, invalid case, branch-change untick, and reset. |
| `ros_nodes/message_converters.py` | `MessageToFields`, `FieldsToMessage` | Dynamic message fields and an exact fields-to-message-to-fields round trip. |
| `ros_nodes/subtree.py` | `Subtree` | Nested tree loading, public IO propagation, nested structure/state/data publication, running/failure/success propagation, reset, untick, and shutdown. |
| `ros_nodes/enum.py` | `EnumFields` | Deferred blocker below. |
| `ros_nodes/action.py` | `Action` | Deferred blocker below; eventually needs acceptance/rejection, feedback, result, abort, timeout/cancel, input changes, reset, untick, and shutdown. |

## Confirmed Production Blockers

Do not add `xfail` tests or change production code as part of this test-only
effort. Report these independently, then add the indicated integration tests
after a production fix lands.

| Class | Source | Problem | Deferred test |
| --- | --- | --- | --- |
| `Convert` | `nodes/maths.py:92-104` | Setup reads undeclared input `clamp`. | Supported conversion output and unsupported conversion setup failure. |
| `SetAttr` | `nodes/setters.py:84-116` | Declares `attr_value` but reads nonexistent input `value`. | Top-level/nested ROS-message attribute replacement. |
| `ThrottleSuccess` | `ros_nodes/throttle.py:134-148` | Compares `Ok(BTNodeState)` directly with raw states, making terminal branches unreachable. | Success suppression, interval expiry, running, and reset. |
| `EnumFields` | `ros_nodes/enum.py:61-84` | Checks a field name against type-keyed `GENERIC_TYPE_MAP`, preventing normal output creation. | Representative ROS-interface constants and outputs. |
| `Action` | `ros_nodes/action.py:496-522` | Builds feedback output fields from result fields instead of feedback fields. | Full controllable action-server lifecycle. |

## Verification Record

Completed after the initial increment:

```text
pre-commit run --files <changed files>  # passed
colcon build --packages-select ros_bt_py  # passed
pytest tests/integration/nodes_isolation/test_constant_passthrough.py -vv
# 4 passed
pytest tests/integration -vv
# 45 passed before the last fixture-only additions
```

Before expanding coverage, rerun the focused module after each new fixture and
the complete integration suite after each stable batch. For any newly exposed
production defect, preserve earlier passing work and report the node, fixture,
expected state/data, actual state/data, and tree-node error rather than
changing the node or suppressing the test.
