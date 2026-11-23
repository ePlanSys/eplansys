Fleet: the two-route depot
==========================

The corridor instance with three robots and two independent routes, north and
south. Every robot must know the state of both routes. A depot rule prevents a
robot dispatched to one junction from reaching the other, so no single robot
can survey both: the work is divided, and the robot that stays behind is
informed by announcement. The policy branches twice, the second branching
nested in each branch of the first.

Mission
-------

.. list-table::
   :header-rows: 0
   :widths: 30 70

   * - Agents
     - ``r1``, ``r2``, ``r3``
   * - Unknowns
     - whether the north route is blocked; whether the south route is blocked
   * - Depot rule
     - a robot at either junction cannot drive to the other
   * - Goal
     - :math:`\bigwedge_{i \in \{1,2,3\}} \mathit{Kw}_{r_i}\,\mathit{blocked\text{-}north}
       \wedge \mathit{Kw}_{r_i}\,\mathit{blocked\text{-}south}` (six conjuncts)

A robot can satisfy at most two conjuncts by its own sensing, and only those
concerning itself; the remaining four require announcements.

Formalisation
-------------

``robot-fleet-depot`` duplicates each mechanism of the corridor domain per
route; the driving events encode the depot rule:

.. code-block:: text

   (:predicates
     (blocked-north)
     (blocked-south)
     (at-north ?i - agent)
     (at-south ?i - agent))

   (:event e-goto-north
     :parameters (?i - agent)
     :precondition (and (not (at-north ?i)) (not (at-south ?i)))
     :effects (at-north ?i))

``inspect-north`` and ``inspect-south`` are semi-private sensing actions; the
four ``report-*`` actions are public announcements whose preconditions require
the announcer to know their content.

.. code-block:: text

   (:agents r1 r2 r3)

   (:init
     (:and
       (:forall (?i - agent) ([C. All] (and (not (at-north ?i)) (not (at-south ?i)))))
       (:forall (?i - agent) ([C. All] (<Kw. ?i> (blocked-north))))
       (:forall (?i - agent) ([C. All] (<Kw. ?i> (blocked-south))))))

Two independent unknowns yield four designated worlds
(:numref:`fig-depot-worlds`).

.. _fig-depot-worlds:

.. graphviz::
   :caption: The four initial worlds. Each agent's accessibility relation is
      the complete relation on them.

   digraph depot_worlds {
     rankdir=TB;
     nodesep=0.7;
     ranksep=0.55;
     node [shape=circle, fixedsize=true, width=0.92, peripheries=2,
           fontname="sans-serif", fontsize=8];
     edge [dir=none, color="#888888", fontname="sans-serif", fontsize=8];

     w0 [label="north: bl.\nsouth: bl."];
     w1 [label="north: bl.\nsouth: cl."];
     w2 [label="north: cl.\nsouth: bl."];
     w3 [label="north: cl.\nsouth: cl."];

     {rank=same; w0; w1;}
     {rank=same; w2; w3;}

     w0 -> w1;
     w2 -> w3;
     w0 -> w2;
     w1 -> w3;
     w0 -> w3 [style=dotted];
     w1 -> w2 [style=dotted];
   }

Grounding
---------

.. code-block:: bash

   ros2 run plansys2_epddl_grounder ground_epddl \
     -d <share>/plansys2_epddl_grounder/examples/robot-fleet-depot-domain.epddl \
     -p <share>/plansys2_epddl_grounder/examples/robot-fleet-depot-problem.epddl \
     -o /tmp/robot-fleet-depot.json

The output is ``plansys2_epistemic_planner/test/tasks/robot-fleet-depot.json``:
3 agents, 8 atoms, 24 grounded actions, 4 initial worlds, all designated.

Policy
------

AO* returns a policy of depth 6 with twelve nodes
(:numref:`fig-depot-policy`): ``r1`` drives north, inspects and announces; then
``r2`` drives south, inspects and announces. The second half occurs once under
each outcome of the first inspection. The two copies are distinct subpolicies:
they are executed in different epistemic states, and their announcements
differ.

.. _fig-depot-policy:

.. graphviz::
   :caption: The depot policy. Sensing nodes are shaded; each leaf
      corresponds to one combination of route states.

   digraph depot_policy {
     rankdir=TB;
     nodesep=0.4;
     ranksep=0.42;
     splines=true;
     node [shape=box, style=rounded, fontname="sans-serif", fontsize=9,
           margin="0.11,0.07", width=1.5];
     edge [fontname="sans-serif", fontsize=8];

     n0  [label="0  goto-north r1"];
     n1  [label="1  inspect-north r1", style="rounded,filled", fillcolor="#eef3fb"];
     n2  [label="2  report-north-blocked r1"];
     n7  [label="7  report-north-clear r1"];
     n3  [label="3  goto-south r2"];
     n8  [label="8  goto-south r2"];
     n4  [label="4  inspect-south r2", style="rounded,filled", fillcolor="#eef3fb"];
     n9  [label="9  inspect-south r2", style="rounded,filled", fillcolor="#eef3fb"];
     n5  [label="5  report-south-blocked r2"];
     n6  [label="6  report-south-clear r2"];
     n10 [label="10  report-south-blocked r2"];
     n11 [label="11  report-south-clear r2"];

     n0 -> n1;
     n1 -> n2  [label="north blocked "];
     n1 -> n7  [label=" north clear"];
     n2 -> n3;
     n7 -> n8;
     n3 -> n4;
     n8 -> n9;
     n4 -> n5  [label="south\nblocked "];
     n4 -> n6  [label=" south\nclear"];
     n9 -> n10 [label="south\nblocked "];
     n9 -> n11 [label=" south\nclear"];
   }

The assignment of robots to routes is found by the search: ``r1``/``r3`` would
cost the same as ``r1``/``r2``. Every path has six actions, three per route, and
no robot senses a fact it has already been told.

Execution
---------

As in :doc:`fleet_corridor`, with the depot task and mapping:

.. code-block:: yaml

   planner:
     ros__parameters:
       plan_solver_plugins: ["EPISTEMIC"]
       EPISTEMIC:
         plugin: "plansys2/EpistemicPlanSolver"
         task_file: "<repo>/plansys2_epistemic_planner/test/tasks/robot-fleet-depot.json"
         action_mapping: "<share>/plansys2_epistemic_planner/examples/mappings/robot-fleet-depot.json"
         strategy: "aostar"
         conditional_plan: "policy"

The PDDL actions take a route parameter, whereas the epistemic domain has one
grounded action per route:

.. code-block:: json

   {
     "goto-north_r1":  {"action": "(goto_junction r1 north)", "duration": 30.0},
     "inspect-south_r2": {"action": "(inspect_corridor r2 south)", "duration": 5.0},
     "report-south-clear_r2": {"action": "(report_clear r2 south)", "duration": 1.0}
   }

Observations are reported per action: the source bound to ``observed`` answers
``(inspect_corridor r1 north)`` and ``(inspect_corridor r2 south)``
independently.

Performance
-----------

.. list-table:: Search on ``robot-fleet-depot.json``; ``aostar``; release
   build.
   :header-rows: 1
   :widths: 34 22 44

   * - Quantity
     - Value
     - Note
   * - Solution depth
     - 6
     - two drives, two inspections, two announcements
   * - Nodes expanded
     - 1 670
     - ~100× the corridor
   * - Nodes generated
     - 2 010
     -
   * - Policy nodes
     - 12
     -
   * - Leaves
     - 4
     - one per combination of route states
   * - Wall clock
     - 0.5 s
     - dominated by process start-up
   * - Peak resident memory
     - 28 MB
     - unchanged from the corridor

``plansys2_tests/test_6`` executes the instance end to end with the routes in
opposite states, which exercises a switch nested in another switch's
continuation.

:doc:`fleet_survey` scales the domain to four robots and three routes.
