Command-line interface
======================

``plansys2_tui_cli`` provides five ``ros2 plansys2`` verbs and a full-screen
dashboard for a running system. Four verbs report on the classical
components; ``epistemic`` queries and updates the ``epistemic_state`` node,
which must be running (``epistemic_state:=True``, :doc:`first_plan`).

Verbs
-----

.. list-table:: ``ros2 plansys2`` verbs.
   :header-rows: 1
   :widths: 34 66

   * - Verb
     - Output
   * - ``knowledge``
     - Instances, predicates, functions and goal of the problem expert.
   * - ``performers``
     - Registered action performers and their status
       (``performers_status``).
   * - ``plan_monitor``
     - The executing plan and the progress of each action
       (``executing_plan``, ``action_execution_info``).
   * - ``execution_monitor``
     - Actions in flight on ``actions_hub``.
   * - ``epistemic``
     - Queries and updates of the epistemic state (below).

The four monitors redraw in place until interrupted. ``--duration`` bounds
their run time; ``knowledge --once`` prints one snapshot. Colour is emitted
only to a terminal and is disabled by ``NO_COLOR``.

``epistemic``
~~~~~~~~~~~~~

.. list-table:: ``ros2 plansys2 epistemic`` commands.
   :header-rows: 1
   :widths: 30 70

   * - Command
     - Semantics
   * - ``show``
     - Agents, atoms, number of worlds and designated worlds, goal and its
       origin. Reads the latched ``epistemic_state/state`` topic; no service
       call.
   * - ``check <formula>``
     - Whether the formula holds. A parse error is reported distinctly from
       a false result. ``--agent`` evaluates from one agent's perspective.
   * - ``goal [<formula>]``
     - Without argument: the goal and whether it holds. With an argument:
       set the goal (parsed against the task; empty restores the task's
       goal).
   * - ``announce <formula>``
     - Public announcement. Reports the number of remaining worlds.
   * - ``apply <action>``
     - Product update by the action's event model. ``--observed`` names the
       outcome when the model designates several worlds.
   * - ``domain``, ``action <name>``
     - Domain and event-model introspection (:ref:`arch-domain`).
   * - ``view <agent>``
     - The model from one agent's perspective (:ref:`arch-perspective`).

Formulas use the syntax of :doc:`../concepts/epddl`, e.g. ``(K r1 blocked)``,
``(Kw r1 blocked)``. Atoms are written bare. ``--timeout`` (default 5 s)
bounds the wait for the state.

Example session
---------------

The corridor scenario of :doc:`../tutorials/fleet_corridor`, driven manually.
Start the state:

.. code-block:: bash

   ros2 run plansys2_epistemic_executor epistemic_state_node --ros-args \
     -p epddl_domain:=<share>/plansys2_epddl_grounder/examples/robot-fleet-domain.epddl \
     -p epddl_problem:=<share>/plansys2_epddl_grounder/examples/robot-fleet-problem.epddl

Initial state:

.. code-block:: console

   $ ros2 plansys2 epistemic show
   Epistemic state:

     agents      2
     atoms       1
     worlds      2
     designated  2

     goal        (and (Kw r1 blocked) (Kw r2 blocked))  (from the task, does not hold)

The model designates two worlds, one with the corridor blocked and one with it
clear. Neither robot knows whether it is blocked:

.. code-block:: console

   $ ros2 plansys2 epistemic check "(Kw r1 blocked)"
   does not hold
   $ ros2 plansys2 epistemic check "(K r1 blocked)"
   does not hold

``inspect_r1`` has two designated events, so the observed one must be given:

.. code-block:: console

   $ ros2 plansys2 epistemic apply inspect_r1 --observed e-inspect-blocked
   applied inspect_r1: observing e-inspect-blocked, 2 worlds, 1 designated
   $ ros2 plansys2 epistemic check "(Kw r1 blocked)"
   holds
   $ ros2 plansys2 epistemic check "(Kw r2 blocked)"
   does not hold

The sensing is semi-private: ``r2`` observes that ``r1`` inspected the corridor
but not the result, so the model retains the world ``r2`` cannot exclude. The
report removes it:

.. code-block:: console

   $ ros2 plansys2 epistemic apply report-blocked_r1
   applied report-blocked_r1: 1 worlds, 1 designated
   $ ros2 plansys2 epistemic check "(Kw r2 blocked)"
   holds
   $ ros2 plansys2 epistemic show | tail -1
     goal        (and (Kw r1 blocked) (Kw r2 blocked))  (from the task, holds)

The state of the corridor did not change during the session; the two updates
changed what ``r1`` and then ``r2`` know about it. A PDDL predicate would have
changed once.

Announcement
~~~~~~~~~~~~

From the initial model, a public announcement reaches the same knowledge
without any action:

.. code-block:: console

   $ ros2 plansys2 epistemic announce blocked
   announced: 1 worlds remain, 1 designated

``apply`` advances the model by an action the planner branched on;
``announce`` adds information from outside the plan. The same distinction
governs how perception reports a region (:ref:`arch-perception`). An
announcement false in every designated world is rejected:

.. code-block:: console

   $ ros2 plansys2 epistemic announce "(not blocked)"
   announcing '(not blocked)' would leave no possible world: it is false everywhere the state considers possible

Changing the goal
~~~~~~~~~~~~~~~~~

The planner plans for the goal the state publishes, so setting it re-targets a
running system without re-grounding:

.. code-block:: console

   $ ros2 plansys2 epistemic goal "(and (K r1 blocked) (K r2 blocked))"
   goal set: (and (K r1 blocked) (K r2 blocked))  (already holds)

An empty argument restores the task's goal.

Dashboard
---------

``plansys2_tui`` shows the four monitors in one Textual application over a
single ROS node:

.. code-block:: bash

   ros2 run plansys2_tui_cli plansys2_tui

.. code-block:: text

   ┌────────────────────┬─────────────────────┐
   │  Action Execution  │  Plan Monitor       │
   │  (actions_hub)     │  (executing_plan +  │
   │                    │   action_exec_info) │
   ├────────────────────┼─────────────────────┤
   │  Performers        │  Knowledge          │
   │  (performers_stat) │  (problem_expert/   │
   │                    │   knowledge)        │
   └────────────────────┴─────────────────────┘

``q`` or ``Ctrl+C`` quits. Quadrants scroll independently; the ROS node is
spun on a separate thread. The dashboard covers the classical components only.

Textual is vendored, so the dashboard runs on a workspace provisioned by
``rosdep`` alone. Importing ``plansys2_tui_cli.tui`` adds the vendored copy to
``sys.path`` and backfills names from ``typing_extensions`` and
``platformdirs``, whose Jammy versions are older than Textual expects.

Package placement
-----------------

The verbs are in ``plansys2_tui_cli``. Placing them in ``plansys2_terminal``
as C++ commands would make the classical terminal depend on
``plansys2_epistemic_msgs``. The Python verb imports those messages at run
time, so ``ros2 plansys2`` works on a workspace without the epistemic packages,
and ``epistemic`` reports the missing dependency when invoked.
