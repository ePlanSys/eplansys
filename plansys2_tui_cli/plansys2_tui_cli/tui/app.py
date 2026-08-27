#!/usr/bin/env python3

# Copyright 2026 Intelligent Robotics Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""PlanSys2 4-quadrant Textual TUI."""

from __future__ import annotations

import atexit
import os
import sys
import threading

# Prefer vendored textual shipped inside this package (no system pip needed).
_vendor_dir = os.path.normpath(
    os.path.join(os.path.dirname(__file__), '..', 'vendor')
)
if os.path.isdir(_vendor_dir) and _vendor_dir not in sys.path:
    sys.path.insert(0, _vendor_dir)

# Imported below the path insertion above rather than at the top of the file,
# which is what E402 flags: `textual` is only importable once the vendored copy
# is on sys.path, and the ROS imports are kept with it so the block reads as one
# thing rather than being split around the reason.
import rclpy  # noqa: E402
from rclpy.executors import ExternalShutdownException  # noqa: E402

from ._typing_compat import patch as _patch_typing_extensions  # noqa: E402

# Humble's typing_extensions is older than the vendored textual expects, so the
# names it imports are backfilled before textual is imported at all.
_patch_typing_extensions()

from textual.app import App, ComposeResult  # noqa: E402
from textual.containers import Horizontal, ScrollableContainer, Vertical  # noqa: E402
from textual.widgets import Footer, Label, Static  # noqa: E402

from ..controller.ros_controllers import (  # noqa: E402
    ActionExecutionProcessor,
    KnowledgeProcessor,
    PerformersProcessor,
    PlanMonitorProcessor,
)


class PlanSys2App(App):
    """
    Four quadrants over one ROS node, laid out as below.

    ┌────────────────────┬─────────────────────┐
    │  Action Execution  │  Plan Monitor       │
    │  (actions_hub)     │  (executing_plan +  │
    │                    │  action_exec_info)  │
    ├────────────────────┼─────────────────────┤
    │  Performers        │  Knowledge          │
    │  (performers_stat) │  (problem_expert/   │
    │                    │   knowledge)        │
    └────────────────────┴─────────────────────┘
    """

    CSS = """
    Screen {
        layout: vertical;
    }

    #grid {
        layout: horizontal;
        width: 100%;
        height: 1fr;
    }

    /* Left column — 50% */
    #left_col {
        width: 50%;
        height: 100%;
        layout: vertical;
    }

    /* Right column — 50% */
    #right_col {
        width: 50%;
        height: 100%;
        layout: vertical;
        margin-left: 1;
    }

    /* Each quadrant occupies half the column height */
    .quadrant {
        height: 1fr;
        layout: vertical;
        margin-bottom: 1;
    }

    .q_title {
        height: 1;
        padding: 0 1;
        background: $panel;
        color: $text;
        text-style: bold;
    }

    .q_box {
        border: round $primary;
        padding: 0 1;
        height: 1fr;
    }

    .q_box > Static {
        width: 100%;
        height: auto;
    }
    """

    BINDINGS = [
        ('q', 'quit', 'Quit'),
        ('ctrl+c', 'quit', 'Quit'),
    ]

    # ------------------------------------------------------------------
    def __init__(self, **kwargs):
        super().__init__(**kwargs)

        rclpy.init(args=None)
        atexit.register(self._ros_shutdown)
        self.node = rclpy.create_node('plansys2_tui')

        # Widget references (set during compose / mount)
        self._box_exec: Static | None = None
        self._box_plan: Static | None = None
        self._box_perf: Static | None = None
        self._box_know: Static | None = None

        # ROS processors
        self._proc_exec = ActionExecutionProcessor(self.node, self._exec_cb)
        self._proc_perf = PerformersProcessor(self.node, self._perf_cb)
        self._proc_plan = PlanMonitorProcessor(self.node, self._plan_cb)
        self._proc_know = KnowledgeProcessor(self.node, self._know_cb)

    # ------------------------------------------------------------------
    def compose(self) -> ComposeResult:
        with Horizontal(id='grid'):
            # ── Left column ──────────────────────────────────────────
            with Vertical(id='left_col'):
                # Top-left: Action Execution
                with Vertical(classes='quadrant'):
                    yield Label(
                        'Action Execution  [performers_status / actions_hub]',
                        classes='q_title',
                    )
                    with ScrollableContainer(classes='q_box'):
                        self._box_exec = Static('[dim]Waiting for actions…[/dim]')
                        yield self._box_exec

                # Bottom-left: Performers
                with Vertical(classes='quadrant'):
                    yield Label(
                        'Performers  [performers_status]', classes='q_title'
                    )
                    with ScrollableContainer(classes='q_box'):
                        self._box_perf = Static('[dim]Waiting for performers…[/dim]')
                        yield self._box_perf

            # ── Right column ─────────────────────────────────────────
            with Vertical(id='right_col'):
                # Top-right: Plan Monitor
                with Vertical(classes='quadrant'):
                    yield Label(
                        'Plan Monitor  [executing_plan / action_execution_info]',
                        classes='q_title',
                    )
                    with ScrollableContainer(classes='q_box'):
                        self._box_plan = Static('[dim]Waiting for plan…[/dim]')
                        yield self._box_plan

                # Bottom-right: Knowledge
                with Vertical(classes='quadrant'):
                    yield Label(
                        'Knowledge  [problem_expert/knowledge]', classes='q_title'
                    )
                    with ScrollableContainer(classes='q_box'):
                        self._box_know = Static('[dim]Waiting for knowledge…[/dim]')
                        yield self._box_know

        yield Footer()

    def on_mount(self) -> None:
        self._ros_thread = threading.Thread(
            target=self._ros_spin, daemon=True
        )
        self._ros_thread.start()

    # ------------------------------------------------------------------
    # ROS background spin
    # ------------------------------------------------------------------
    def _ros_spin(self) -> None:
        try:
            rclpy.spin(self.node)
        except (KeyboardInterrupt, ExternalShutdownException):
            pass

    # ------------------------------------------------------------------
    # ROS callbacks
    # ------------------------------------------------------------------
    def _exec_cb(self, performers: dict, active: dict) -> None:
        if self._box_exec is None:
            return
        text = ActionExecutionProcessor.combined2text(performers, active)
        self.call_from_thread(self._box_exec.update, text)

    def _perf_cb(self, performers: dict) -> None:
        if self._box_perf is None:
            return
        text = PerformersProcessor.performers2text(performers)
        self.call_from_thread(self._box_perf.update, text)

    def _plan_cb(self, plan, info: dict) -> None:
        if self._box_plan is None:
            return
        text = PlanMonitorProcessor.plan2text(plan, info)
        self.call_from_thread(self._box_plan.update, text)

    def _know_cb(self, msg) -> None:
        if self._box_know is None:
            return
        text = KnowledgeProcessor.knowledge2text(msg)
        self.call_from_thread(self._box_know.update, text)

    # ------------------------------------------------------------------
    # Shutdown
    # ------------------------------------------------------------------
    def _ros_shutdown(self) -> None:
        if rclpy.ok():
            try:
                self.node.destroy_node()
            except Exception:
                pass
            rclpy.shutdown()
        if hasattr(self, '_ros_thread'):
            self._ros_thread.join(timeout=1.0)


# ------------------------------------------------------------------
def run_app() -> None:
    """Entry point for the ``plansys2_tui`` console script."""
    PlanSys2App().run()


if __name__ == '__main__':
    run_app()
