Fleet: the blocked corridor
===========================

The smallest instance of the fleet family: two robots, one corridor, one
sensing action, one branch. It introduces every construct used in
:doc:`fleet_depot` and :doc:`fleet_survey`.

Mission
-------

.. list-table::
   :header-rows: 0
   :widths: 30 70

   * - Agents
     - ``r1``, ``r2``; symmetric survey robots
   * - Unknown
     - whether the corridor is blocked
   * - Actions
     - drive to the junction; inspect the corridor; announce the result
   * - Goal
     - :math:`\mathit{Kw}_{r_1}\,\mathit{blocked} \wedge
       \mathit{Kw}_{r_2}\,\mathit{blocked}`

The goal is a knowing-whether goal. A goal of :math:`K\,\mathit{blocked}`
would be unachievable in the world where the corridor is clear.

Formalisation
-------------

The domain ``robot-fleet`` is in
``plansys2_epddl_grounder/examples/robot-fleet-domain.epddl``. It has two
predicates:

.. code-block:: text

   (:predicates
     (blocked)                    ; the corridor is obstructed
     (at-junction ?i - agent)     ; ?i can observe the corridor
   )

The three actions differ only in their action type and observability:

.. code-block:: text

   (:action goto-junction
     :parameters (?i - agent)
     :action-type (public-ontic (e-goto-junction ?i))
     :observability-conditions (default Fully))

   (:action inspect
     :parameters (?i - agent)
     :action-type (semi-private-sensing (e-inspect-blocked ?i) (e-inspect-clear ?i))
     :observability-conditions (:and (?i Fully) (default Partially)))

   (:action report-blocked
     :parameters (?i - agent)
     :action-type (public-announcement (e-knows-blocked ?i) (e-not-knows-blocked ?i))
     :observability-conditions (default Fully))

``report-clear`` is the counterpart of ``report-blocked``. Each announcement
requires the announcer to know its content, so the domain has one action per
result.

.. _fig-observability:

.. graphviz::
   :caption: Effect of each action type on the agents' knowledge.

   digraph observability {
     rankdir=TB;
     nodesep=0.3;
     ranksep=0.45;
     fontname="sans-serif";
     node [fontname="sans-serif", fontsize=9, shape=box, style=rounded, margin="0.12,0.07"];
     edge [fontname="sans-serif", fontsize=8, arrowsize=0.7];

     subgraph cluster_ontic {
       label="public-ontic:  goto-junction r1";
       labelloc=t; fontsize=10; style=dashed; color="#aaaaaa"; margin=12;
       o_act [label="r1 drives", style="rounded,filled", fillcolor="#eef3fb"];
       o_all [label="r1, r2 know\nr1 is at the junction"];
       o_act -> o_all [label=" world changes,\l all observe it\l"];
     }

     subgraph cluster_sense {
       label="semi-private-sensing:  inspect r1";
       labelloc=t; fontsize=10; style=dashed; color="#aaaaaa"; margin=12;
       s_act [label="r1 inspects", style="rounded,filled", fillcolor="#eef3fb"];
       s_r1  [label="r1 knows whether\nthe corridor is blocked"];
       s_r2  [label="r2 knows only\nthat r1 inspected"];
       s_act -> s_r1 [label=" Fully"];
       s_act -> s_r2 [label=" Partially"];
     }

     subgraph cluster_say {
       label="public-announcement:  report-blocked r1";
       labelloc=t; fontsize=10; style=dashed; color="#aaaaaa"; margin=12;
       a_act [label="r1 announces", style="rounded,filled", fillcolor="#eef3fb"];
       a_all [label="r1, r2 know\nthe corridor is blocked"];
       a_act -> a_all [label=" truthful announcement\l"];
     }

     o_all -> s_act [style=invis];
     s_r2  -> a_act [style=invis];
   }

The sensing is semi-private (:numref:`fig-observability`). Public sensing would
inform ``r2`` directly and make the announcement unnecessary; private sensing
would leave ``r2`` unaware that an inspection took place. A camera on one robot
corresponds to the semi-private case: the other robot observes the behaviour,
not the image.

Initial theory
~~~~~~~~~~~~~~

.. code-block:: text

   (:agents r1 r2)

   (:init
     (:and
       (:forall (?i - agent) ([C. All] (not (at-junction ?i))))
       (:forall (?i - agent) ([C. All] (<Kw. ?i> (blocked))))))

   (:goal
     (and ([Kw. r1] (blocked)) ([Kw. r2] (blocked))))

The theory does not determine ``blocked``. It is satisfied by two worlds, both
designated, so the initial state is multi-pointed and every solution branches
(:numref:`fig-corridor-models`).

.. _fig-corridor-models:

.. graphviz::
   :caption: The model along one execution. Double circles are designated
      worlds; an edge labelled with an agent means the agent cannot
      distinguish its endpoints.

   digraph corridor_models {
     rankdir=LR;
     compound=true;
     nodesep=0.34;
     ranksep=0.55;
     fontname="sans-serif";
     node [shape=circle, fixedsize=true, width=0.66, fontname="sans-serif", fontsize=8];
     edge [fontname="sans-serif", fontsize=8, dir=none];

     subgraph cluster_a {
       label="initial"; fontsize=10; labelloc=b; style=dashed; color="#999999"; margin=10;
       a_b [label="blocked", peripheries=2];
       a_c [label="clear", peripheries=2];
       a_b -> a_c [label="r1, r2"];
     }

     subgraph cluster_b {
       label="after inspect r1"; fontsize=10; labelloc=b; style=dashed; color="#999999"; margin=10;
       b_b [label="blocked", peripheries=2];
       b_c [label="clear"];
       b_b -> b_c [label="r2"];
     }

     subgraph cluster_c {
       label="after report"; fontsize=10; labelloc=b; style=dashed; color="#999999"; margin=10;
       c_b [label="blocked", peripheries=2];
       c_c [label="clear", style=dotted, fontcolor="#888888", color="#aaaaaa"];
     }

     a_c -> b_b [ltail=cluster_a, lhead=cluster_b, dir=forward, style=bold, label="inspect r1"];
     b_c -> c_b [ltail=cluster_b, lhead=cluster_c, dir=forward, style=bold, label="report-blocked r1"];
   }

The goal holds when no agent has an edge between a blocked and a clear world.

Grounding
---------

.. code-block:: bash

   ros2 run plansys2_epddl_grounder ground_epddl \
     -d <share>/plansys2_epddl_grounder/examples/robot-fleet-domain.epddl \
     -p <share>/plansys2_epddl_grounder/examples/robot-fleet-problem.epddl \
     -o /tmp/robot-fleet.json

The output is ``plansys2_epistemic_planner/test/tasks/robot-fleet.json``: 2
agents, 3 atoms, 8 grounded actions, 2 initial worlds, both designated. The
solver grounds automatically when given ``epddl_domain`` and ``epddl_problem``.

Policy
------

AO* returns a policy of depth 3 with four nodes (:numref:`fig-corridor-policy`).

.. _fig-corridor-policy:

.. graphviz::
   :caption: The corridor policy. The branch is at the sensing action.

   digraph corridor_policy {
     rankdir=TB;
     nodesep=0.55;
     ranksep=0.45;
     node [shape=box, style=rounded, fontname="sans-serif", fontsize=10, margin="0.14,0.09"];
     edge [fontname="sans-serif", fontsize=9];

     n0 [label="0  goto-junction r1"];
     n1 [label="1  inspect r1\n(sensing)", style="rounded,filled", fillcolor="#eef3fb"];
     n2 [label="2  report-blocked r1\nneeds K(r1, blocked)"];
     n3 [label="3  report-clear r1\nneeds K(r1, ¬blocked)"];
     d2 [label="goal holds", shape=ellipse, style=solid];
     d3 [label="goal holds", shape=ellipse, style=solid];

     n0 -> n1 [label="  e-goto-junction"];
     n1 -> n2 [label="  e-inspect-blocked  "];
     n1 -> n3 [label="  e-inspect-clear"];
     n2 -> d2 [label="  e-knows-blocked  "];
     n3 -> d3 [label="  e-knows-clear"];
   }

The knowledge requirements of the announcements,
:math:`K_{r_1}\,\mathit{blocked}` and :math:`K_{r_1}\neg\mathit{blocked}`, have
no PDDL representation. At execution time ``CheckKnowledge`` evaluates them
against the epistemic state, which prevents an announcement the robot cannot
justify.

Both continuations are part of the policy; the observed outcome selects one at
run time. A flattened plan (``conditional_plan: flatten``) contains only one
and is valid only in the corresponding world.

The alternative of both robots driving out and inspecting costs four actions;
the returned policy costs three, replacing the second inspection with an
announcement.

Execution
---------

Planner
   .. code-block:: yaml

      planner:
        ros__parameters:
          plan_solver_plugins: ["EPISTEMIC"]
          EPISTEMIC:
            plugin: "plansys2/EpistemicPlanSolver"
            task_file: "<repo>/plansys2_epistemic_planner/test/tasks/robot-fleet.json"
            action_mapping: "<share>/plansys2_epistemic_planner/examples/mappings/robot-fleet.json"
            strategy: "aostar"
            conditional_plan: "policy"

   ``conditional_plan: "policy"`` keeps the branches; the default
   ``"flatten"`` targets a stock PlanSys2 executor.

Action mapping
   .. code-block:: json

      {
        "goto-junction_r1": {"action": "(goto_junction r1)", "duration": 30.0},
        "inspect_r1":       {"action": "(inspect_corridor r1)", "duration": 5.0},
        "report-blocked_r1": {"action": "(report_blocked r1)", "duration": 1.0},
        "report-clear_r1":   {"action": "(report_clear r1)", "duration": 1.0}
      }

   Durations are properties of the action implementations, used by the
   executor for scheduling and timeouts.

Executor
   .. code-block:: yaml

      executor:
        ros__parameters:
          bt_builder_plugin: "EpistemicBTBuilder"
          bt_node_plugins: ["libplansys2_epistemic_bt_nodes.so"]

Epistemic state
   Loaded from the same task as the planner, so that knowledge requirements
   are evaluated in the same vocabulary:

   .. code-block:: bash

      ros2 run plansys2_epistemic_executor epistemic_state_node --ros-args \
        -p task_file:=<repo>/plansys2_epistemic_planner/test/tasks/robot-fleet.json

Observation
   With two designated worlds the state cannot determine the outcome of
   ``inspect_r1``; the ``inspect_corridor`` performer reports it in
   ``ActionExecution.outcome`` (:ref:`arch-observation`):

   .. code-block:: cpp

      finish(true, 1.0, "Corridor inspected", clear ? "e-inspect-clear"
                                                    : "e-inspect-blocked");

   The packaged action template passes the value from ``ExecuteAction`` to
   ``ApplyEpistemicUpdate``. An observation from another source is supplied
   by binding the ``observed`` port in a custom template
   (:doc:`../reports/epistemic_end_to_end`).

Performance
-----------

.. list-table:: Search on ``robot-fleet.json``; ``aostar`` with the heuristic
   chosen by the selection policy (``ks``); release build.
   :header-rows: 1
   :widths: 34 22 44

   * - Quantity
     - Value
     - Note
   * - Solution depth
     - 3
     - drive, inspect, report
   * - Nodes expanded
     - 17
     -
   * - Nodes generated
     - 19
     -
   * - Policy nodes
     - 4
     -
   * - Leaves
     - 2
     - one per world
   * - Wall clock
     - 0.5 s
     - dominated by process start-up
   * - Peak resident memory
     - 28 MB
     - dominated by the ROS process

``plansys2_tests/test_5`` executes the instance end to end over a live node
graph, once with the corridor blocked and once clear.

:doc:`fleet_depot` extends the instance to two corridors and three robots, with
division of labour and nested branching.
