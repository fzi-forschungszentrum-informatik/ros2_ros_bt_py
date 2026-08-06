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

from __future__ import annotations

import contextlib
import os
from pathlib import Path
import re
import signal
import socket
import subprocess
import time

import domain_coordinator
from playwright.sync_api import Page, expect
import pytest
from ros_bt_py_interfaces.srv import ControlTreeExecution


ARTIFACT_ROOT = Path(os.environ.get("ROS_BT_PY_UI_ARTIFACT_DIR", "test-results/web_ui"))


def pytest_configure(config: pytest.Config) -> None:
    """Configure pytest-playwright diagnostics for this test directory."""
    config.option.tracing = "retain-on-failure"
    config.option.screenshot = "only-on-failure"
    config.option.output = str(ARTIFACT_ROOT)


@pytest.fixture(scope="session")
def browser_type_launch_args(browser_type_launch_args):
    """Allow local runs to use an installed Chrome executable."""
    executable_path = os.environ.get("PLAYWRIGHT_CHROMIUM_EXECUTABLE")
    browser_channel = os.environ.get("PLAYWRIGHT_BROWSER_CHANNEL")
    if executable_path:
        return {**browser_type_launch_args, "executable_path": executable_path}
    if browser_channel:
        return {**browser_type_launch_args, "channel": browser_channel}
    return browser_type_launch_args


@pytest.fixture(scope="module")
def web_gui_stack():
    with contextlib.ExitStack() as stack:
        if (
            "ROS_DOMAIN_ID" not in os.environ
            and "DISABLE_ROS_ISOLATION" not in os.environ
        ):
            domain_id = stack.enter_context(domain_coordinator.domain_id())
            os.environ["ROS_DOMAIN_ID"] = str(domain_id)

        log_path = ARTIFACT_ROOT / "web_gui_stack.log"
        log_path.parent.mkdir(parents=True, exist_ok=True)
        log_file = stack.enter_context(log_path.open("w", encoding="utf-8"))
        process = subprocess.Popen(
            [
                "ros2",
                "launch",
                "ros_bt_py",
                "ros_bt_py.launch.py",
                "enable_web_interface:=True",
                "load_default_tree:=False",
                "robot_namespace:=/",
            ],
            env=os.environ.copy(),
            stdout=log_file,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        try:
            yield process
        finally:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGTERM)
                    process.wait(timeout=15)


@pytest.fixture(autouse=True)
def browser_diagnostics(page: Page, request: pytest.FixtureRequest):
    """Fail on browser/runtime failures and preserve useful diagnostics."""
    errors: list[str] = []

    def record(message: str) -> None:
        errors.append(message)

    page.on("pageerror", lambda error: record(f"pageerror: {error}"))
    page.on("crash", lambda _page: record("page crashed"))
    page.on(
        "console",
        lambda message: (
            record(f"console.error: {message.text}")
            if message.type == "error"
            else None
        ),
    )
    page.on(
        "requestfailed",
        lambda request_: (
            record(f"requestfailed: {request_.url} ({request_.failure})")
            if request_.failure != "net::ERR_CONNECTION_REFUSED"
            else None
        ),
    )

    def handle_dialog(dialog) -> None:
        record(f"unexpected dialog: {dialog.type}: {dialog.message}")
        dialog.dismiss()

    page.on("dialog", handle_dialog)

    def check_response(response) -> None:
        if response.status >= 400 and response.request.resource_type in {
            "document",
            "script",
            "stylesheet",
        }:
            record(f"HTTP {response.status}: {response.url}")

    page.on("response", check_response)

    yield

    if not errors:
        return

    artifact_name = re.sub(r"[^A-Za-z0-9_.-]+", "_", request.node.nodeid)
    artifact_dir = ARTIFACT_ROOT / artifact_name
    artifact_dir.mkdir(parents=True, exist_ok=True)
    (artifact_dir / "browser-errors.log").write_text(
        "\n".join(errors), encoding="utf-8"
    )
    try:
        page.screenshot(path=str(artifact_dir / "failure.png"), full_page=True)
        (artifact_dir / "page.html").write_text(page.content(), encoding="utf-8")
    except Exception as error:  # pragma: no cover - only used during failures
        (artifact_dir / "diagnostic-error.log").write_text(str(error), encoding="utf-8")
    pytest.fail("Browser runtime failures:\n" + "\n".join(errors))


@pytest.fixture(autouse=True)
def reset_tree_execution(tree_control_node):
    yield
    tree_control_node.execute_tree(ControlTreeExecution.Request.SHUTDOWN)


@pytest.fixture
def open_web_gui(page: Page, web_gui_stack: subprocess.Popen, tree_control_node):
    """Open the shipped GUI and wait for its real ROS connection."""
    if web_gui_stack.poll() is not None:
        raise AssertionError(
            f"ros_bt_py launch exited; see {ARTIFACT_ROOT / 'web_gui_stack.log'}"
        )
    if not tree_control_node.load_tree_client.wait_for_service(timeout_sec=30):
        raise AssertionError("tree_node services did not become available")

    deadline = time.monotonic() + 30
    while time.monotonic() < deadline:
        try:
            with socket.create_connection(("127.0.0.1", 9090), timeout=0.5):
                break
        except OSError:
            time.sleep(0.1)
    else:
        raise AssertionError("rosbridge websocket did not become available")

    page.add_init_script(
        "if (!window.localStorage.getItem('ros')) window.localStorage.clear()"
    )
    page.goto(
        "http://127.0.0.1:8085/index.html",
        wait_until="domcontentloaded",
        timeout=30_000,
    )
    settings_button = page.locator('button[data-bs-target="#settings"]')
    settings_button.click()
    expect(page.locator("#settings")).to_be_visible(timeout=30_000)
    page.locator("svg.connected, svg.messages-missing, svg.packages-missing").wait_for(
        state="visible", timeout=30_000
    )
    namespace_select = page.locator("#settings select").nth(1)
    expect(namespace_select).to_have_value("/BehaviorTreeNode/", timeout=30_000)
    namespace_select.select_option("/BehaviorTreeNode/")
    namespace_select.dispatch_event("change")
    page.reload(wait_until="domcontentloaded", timeout=30_000)
    page.locator("svg.connected, svg.messages-missing, svg.packages-missing").wait_for(
        state="visible", timeout=30_000
    )
    if page.locator("#settings").is_visible():
        page.locator('button[data-bs-target="#settings"]').click()

    return page
