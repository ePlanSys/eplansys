Fleet: the three-route survey
=============================

The largest instance of the family: four robots and three independent routes.
The actions are those of :doc:`fleet_corridor` and :doc:`fleet_depot` without
additions, so the three instances differ only in the size of the search.

The instance is not part of the test suite; its search time (about one minute)
exceeds what per-commit CI allows. The two smaller instances provide the
regression tests.

Mission
-------

.. list-table::
   :header-rows: 0
   :widths: 30 70

   * - Agents
     - ``r1``, ``r2``, ``r3``, ``r4``
   * - Unknowns
     - whether each of north, south and east is blocked
   * - Depot rule
     - a robot dispatched to a junction remains there
   * - Goal
     - twelve conjuncts :math:`\mathit{Kw}_{r_i}\,\mathit{blocked\text{-}d}`,
       :math:`i \in \{1..4\}`, :math:`d \in \{\mathit{north}, \mathit{south},
       \mathit{east}\}`

Each robot can resolve at most one route by sensing, so every solution needs
three inspections and three announcements.

Formalisation
-------------

``robot-fleet-survey`` is the depot domain with a third route; the problem adds
a fourth agent. The file is generated from the same template as the depot
domain, which carries the commentary.

.. code-block:: text

   (:predicates
     (blocked-north) (blocked-south) (blocked-east)
     (at-north ?i - agent) (at-south ?i - agent) (at-east ?i - agent))

This gives 15 atoms (three route states and three positions per robot) and 48
grounded actions (per route: four drive, four inspect and eight report
actions). The initial theory leaves all three routes open, so the model
designates eight worlds:

.. code-block:: text

   (:agents r1 r2 r3 r4)

   (:init
     (:and
       (:forall (?i - agent) ([C. All] (not (at-north ?i))))
       (:forall (?i - agent) ([C. All] (not (at-south ?i))))
       (:forall (?i - agent) ([C. All] (not (at-east ?i))))
       (:forall (?i - agent) ([C. All] (<Kw. ?i> (blocked-north))))
       (:forall (?i - agent) ([C. All] (<Kw. ?i> (blocked-south))))
       (:forall (?i - agent) ([C. All] (<Kw. ?i> (blocked-east))))))

Grounding
---------

The grounded task is not checked in:

.. code-block:: bash

   ros2 run plansys2_epddl_grounder ground_epddl \
     -d <share>/plansys2_epddl_grounder/examples/robot-fleet-survey-domain.epddl \
     -p <share>/plansys2_epddl_grounder/examples/robot-fleet-survey-problem.epddl \
     -o /tmp/robot-fleet-survey.json

Grounding takes under 0.1 s; ``planning-task-info`` reports 4 agents, 15 atoms,
48 actions and 8 initial worlds. The action mapping is checked in at
``plansys2_epistemic_planner/examples/mappings/robot-fleet-survey.json``.

Policy
------

AO* returns a policy of depth 9 with 28 nodes and eight leaves. Each survey trip
is drive, inspect, announce, and the policy branches at each inspection
(:numref:`fig-survey-shape`).

.. _fig-survey-shape:

.. graphviz::
   :caption: Sensing skeleton of the survey policy. Each edge also contains the
      drive preceding the inspection and the announcement following it. Leaves
      give the route states (B blocked, C clear) in sensing order: east,
      north, south.

   digraph survey_shape {
     rankdir=TB;
     nodesep=0.22;
     ranksep=0.5;
     node [fontname="sans-serif", fontsize=9];
     edge [fontname="sans-serif", fontsize=8, arrowsize=0.7];

     e  [label="inspect east\nr1", shape=box, style="rounded,filled", fillcolor="#eef3fb"];
     n0 [label="inspect north\nr2", shape=box, style="rounded,filled", fillcolor="#eef3fb"];
     n1 [label="inspect north\nr2", shape=box, style="rounded,filled", fillcolor="#eef3fb"];
     s0 [label="inspect south\nr3", shape=box, style="rounded,filled", fillcolor="#eef3fb"];
     s1 [label="inspect south\nr3", shape=box, style="rounded,filled", fillcolor="#eef3fb"];
     s2 [label="inspect south\nr3", shape=box, style="rounded,filled", fillcolor="#eef3fb"];
     s3 [label="inspect south\nr3", shape=box, style="rounded,filled", fillcolor="#eef3fb"];

     l0 [label="B B B", shape=ellipse, width=0.62, height=0.32, fixedsize=true];
     l1 [label="B B C", shape=ellipse, width=0.62, height=0.32, fixedsize=true];
     l2 [label="B C B", shape=ellipse, width=0.62, height=0.32, fixedsize=true];
     l3 [label="B C C", shape=ellipse, width=0.62, height=0.32, fixedsize=true];
     l4 [label="C B B", shape=ellipse, width=0.62, height=0.32, fixedsize=true];
     l5 [label="C B C", shape=ellipse, width=0.62, height=0.32, fixedsize=true];
     l6 [label="C C B", shape=ellipse, width=0.62, height=0.32, fixedsize=true];
     l7 [label="C C C", shape=ellipse, width=0.62, height=0.32, fixedsize=true];

     e -> n0 [label="blocked "];
     e -> n1 [label=" clear"];
     n0 -> s0 [label="bl. "];
     n0 -> s1 [label=" cl."];
     n1 -> s2 [label="bl. "];
     n1 -> s3 [label=" cl."];
     s0 -> l0; s0 -> l1;
     s1 -> l2; s1 -> l3;
     s2 -> l4; s2 -> l5;
     s3 -> l6; s3 -> l7;
   }

The returned order is east, north, south, with ``r1``, ``r2`` and ``r3`` taking
one route each. ``r4`` performs no action; the four goal conjuncts concerning
``r4`` are satisfied by announcements alone.

Execution
---------

.. code-block:: yaml

   planner:
     ros__parameters:
       plan_solver_timeout: 120.0
       plan_solver_plugins: ["EPISTEMIC"]
       EPISTEMIC:
         plugin: "plansys2/EpistemicPlanSolver"
         task_file: "/tmp/robot-fleet-survey.json"
         action_mapping: "<share>/plansys2_epistemic_planner/examples/mappings/robot-fleet-survey.json"
         strategy: "aostar"
         conditional_plan: "policy"

``plan_solver_timeout`` defaults to 15 s, which this instance exceeds; with the
default the search is abandoned and no plan is returned.

Performance
-----------

.. list-table:: Search on the survey task; ``aostar`` with the heuristic chosen
   by the selection policy (``ks``); release build.
   :header-rows: 1
   :widths: 34 22 44

   * - Quantity
     - Value
     - Note
   * - Solution depth
     - 9
     - three drives, three inspections, three announcements
   * - Nodes expanded
     - 5 029 063
     - ~3000× the depot
   * - Nodes generated
     - 5 746 557
     -
   * - Policy nodes
     - 28
     - seven sensing nodes
   * - Leaves
     - 8
     - one per combination of route states
   * - Wall clock
     - 57 s
     - search-dominated
   * - Peak resident memory
     - 28 MB
     -

.. _tab-fleet-family:

.. list-table:: The fleet family.
   :header-rows: 1
   :widths: 26 14 14 14 16 16

   * - Instance
     - Agents
     - Actions
     - Worlds
     - Expanded
     - Wall clock
   * - :doc:`corridor <fleet_corridor>`
     - 2
     - 8
     - 2
     - 17
     - 0.5 s
   * - :doc:`depot <fleet_depot>`
     - 3
     - 24
     - 4
     - 1 670
     - 0.5 s
   * - survey
     - 4
     - 48
     - 8
     - 5 029 063
     - 57 s

Each additional route doubles the number of designated worlds and adds three to
the solution depth; expansions grow exponentially in depth (17, 1 670,
5.0 × 10\ :sup:`6`).
