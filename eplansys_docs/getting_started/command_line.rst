The command line
================

``plansys2_tui_cli`` is the front end to a running system: five ``ros2
plansys2`` verbs and a full-screen dashboard. Four of the verbs report on the
classical half, the plan and who is executing it; the fifth, ``epistemic``, is
the terminal for the half no predicate can express, and is the quickest way to
watch a Kripke model change without writing a node.

Everything on this page talks to nodes that are already running. Bring a system
up as in :doc:`first_plan` first, and remember ``epistemic_state:=True``, since
the ``epistemic`` verb has nothing to talk to without it.

The verbs
---------

.. list-table::
   :header-rows: 1
   :widths: 34 66

   * - Verb
     - What it shows
   * - ``ros2 plansys2 knowledge``
     - The problem expert's instances, predicates, functions and goal: the
       classical facts, redrawn as they change.
   * - ``ros2 plansys2 performers``
     - The registered action performers and their status, from
       ``performers_status``.
   * - ``ros2 plansys2 plan_monitor``
     - The plan being executed and each action's progress, from
       ``executing_plan`` and ``action_execution_info``.
   * - ``ros2 plansys2 execution_monitor``
     - The actions currently in flight on ``actions_hub``.
   * - ``ros2 plansys2 epistemic``
     - What the agents know: the shape of the model, the goal, whether a
       formula holds, and the two ways of changing it.

The four monitors redraw in place until interrupted. ``--duration`` bounds how
long they run, and ``--once`` on ``knowledge`` prints a single snapshot and
exits, which is the form to use in a script. Colour is emitted only to a
terminal, and ``NO_COLOR`` in the environment turns it off there too, so piping
any of them to a file yields plain text.

The ``epistemic`` verb
----------------------

Five commands, one per question worth asking of the model:

.. list-table::
   :header-rows: 1
   :widths: 30 70

   * - Command
     - Meaning
   * - ``show``
     - The latched state: agents, atoms, how many worlds the model holds and
       how many of them are designated, and the goal with its provenance.
       This one reads the ``epistemic_state/state`` topic instead of calling
       a service, so it costs the node nothing.
   * - ``check <formula>``
     - Does this hold now? A parse failure and a false answer are reported
       differently, so a typo in a formula never reads as knowledge the fleet
       does not have.
   * - ``goal [<formula>]``
     - With no argument, the goal and whether it holds; with one, aim at
       something else. The formula is parsed against the loaded task, so a
       goal naming an agent or an atom the task does not have is refused
       here, before it can fail inside a planning request.
   * - ``announce <formula>``
     - Everyone just learned this. The model is restricted to the worlds where
       the formula holds, and the reply says how many survived.
   * - ``apply <action>``
     - This action ran: advance the model by its event model. ``--observed``
       names the outcome when the model designates several worlds and so
       cannot work out which one occurred.

Formulas are written in the model's own syntax, not EPDDL's: ``(K r1
blocked)``, ``(Kw r1 blocked)``, ``(B r1 blocked)``, and ``(not ...)``,
``(and ...)``, ``(or ...)`` around them. ``--timeout`` bounds the wait for the
state, and defaults to five seconds.

A worked session
----------------

The corridor scenario from :doc:`../tutorials/fleet_corridor`, driven by hand.
Two robots at a depot, a corridor that may be blocked, and nobody has looked.
Start the state on that problem:

.. code-block:: bash

   ros2 run plansys2_epistemic_executor epistemic_state_node --ros-args \
     -p epddl_domain:=<share>/plansys2_epddl_grounder/examples/robot-fleet-domain.epddl \
     -p epddl_problem:=<share>/plansys2_epddl_grounder/examples/robot-fleet-problem.epddl

Ask it what it holds:

.. code-block:: console

   $ ros2 plansys2 epistemic show
   Epistemic state:

     agents      2
     atoms       1
     worlds      2
     designated  2

     goal        (and (Kw r1 blocked) (Kw r2 blocked))  (from the task, does not hold)

Two designated worlds is the whole scenario in one line: the corridor really is
one way or the other, and the model cannot say which. That is why the goal is a
knowing-whether goal, and why the solution has to branch.

Nobody knows anything about the corridor yet, and the model says so twice over:

.. code-block:: console

   $ ros2 plansys2 epistemic check "(Kw r1 blocked)"
   does not hold
   $ ros2 plansys2 epistemic check "(K r1 blocked)"
   does not hold

Now run the sensing action. Because the model designates two worlds it cannot
determine the outcome itself, so name the one that occurred:

.. code-block:: console

   $ ros2 plansys2 epistemic apply inspect_r1 --observed e-inspect-blocked
   applied inspect_r1: observing e-inspect-blocked, 2 worlds, 1 designated

One designated world: r1 looked, and the corridor was blocked. The model still
holds two worlds, though, and the reason is the whole of the next answer:

.. code-block:: console

   $ ros2 plansys2 epistemic check "(Kw r1 blocked)"
   holds
   $ ros2 plansys2 epistemic check "(Kw r2 blocked)"
   does not hold

That second answer is the point of the scenario. Sensing here is semi-private:
r2 watched r1 look and learned *that* it looked, not *what* it saw. The goal is
about both robots, so the mission is not over.

Saying it out loud is what finishes it:

.. code-block:: console

   $ ros2 plansys2 epistemic apply report-blocked_r1
   applied report-blocked_r1: 1 worlds, 1 designated
   $ ros2 plansys2 epistemic check "(Kw r2 blocked)"
   holds
   $ ros2 plansys2 epistemic show | tail -1
     goal        (and (Kw r1 blocked) (Kw r2 blocked))  (from the task, holds)

The second world is gone, and nothing about the corridor changed to remove it.
It was the world r2 could not rule out, and what removed it was being told. A
predicate would have moved on the first line of this session, when r1 looked;
the model moved twice, once for each robot, which is the distinction between a
fact and knowledge of it that the epistemic state exists to keep.

``announce`` reaches the same place from outside a plan. Rather than replaying
r1's report, an operator who has been told the corridor is blocked can put that
into the model directly. From the initial two-world model:

.. code-block:: console

   $ ros2 plansys2 epistemic announce blocked
   announced: 1 worlds remain, 1 designated

Both robots know it at once, and no one had to drive anywhere. Atoms are
written bare, as ``blocked`` and not ``(blocked)``; parentheses introduce
an operator, and ``(blocked)`` is read as one that does not exist.

The two are not interchangeable, and choosing between them is a modelling
decision, not a matter of taste: ``apply`` advances the model by an
action the planner branched on, and ``announce`` is information that arrived
outside the plan. That is the same distinction perception draws when it decides
whether a region reports as a sensing outcome or as an announcement; see
:doc:`../concepts/architecture`.

An announcement that holds in no world the state considers possible is
refused, since applying it would empty the model:

.. code-block:: console

   $ ros2 plansys2 epistemic announce "(not blocked)"
   announcing '(not blocked)' would leave no possible world: it is false everywhere the state considers possible

Changing the goal without re-grounding the problem is the other thing worth
doing from a terminal. The planner subscribes to the latched state and plans
for the goal it finds there, so this is enough to re-aim a running system:

.. code-block:: console

   $ ros2 plansys2 epistemic goal "(and (K r1 blocked) (K r2 blocked))"
   goal set: (and (K r1 blocked) (K r2 blocked))  (already holds)

An empty argument restores the task's own goal.

The dashboard
-------------

``plansys2_tui`` is the four monitors at once, over a single ROS node, in a
Textual full-screen application:

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

``q`` or ``Ctrl+C`` quits. Each quadrant scrolls on its own, and the ROS node
is spun on a thread of its own so a redraw never blocks a subscription.

Textual is vendored inside the package instead of installed with pip, so the
dashboard runs on a workspace built with nothing but ``rosdep``. Importing
``plansys2_tui_cli.tui`` puts the vendored copy on ``sys.path`` and backfills
the names it expects from ``typing_extensions`` and ``platformdirs``, which
Jammy, and therefore Humble, ships older than it wants. The dashboard shows the
classical half only; what the agents know is the ``epistemic`` verb's subject
and has no quadrant.

Why the verbs live here
-----------------------

``plansys2_terminal`` would be the obvious home for ``epistemic``, and is the
wrong one. That package is built by every distribution's workflow, and reaching
the epistemic services from C++ would make the classical terminal depend on the
epistemic message package to do so. A Python verb imports those messages at run
time instead and costs the classical build nothing: ``ros2 plansys2`` keeps
working on a workspace where the epistemic packages were never built, and the
one verb that needs them says what is missing when it is the one invoked.
