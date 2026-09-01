Fleet: the three-route survey
=============================

The fleet scenario at its third size: four robots, three routes out of the
depot, any of which may be blocked, and every robot needing to know the state
of all three.

It is deliberately big, not clever. The mechanisms are those of
:doc:`fleet_corridor` and :doc:`fleet_depot` (drive, look, say what you saw)
with nothing added, so that what grows between the three scenarios is the
search and not the modelling. That is what makes the family worth having: the
same domain at three sizes says something about the planner that no single
instance can.

This one is not part of the test suite. A minute of search is too long for a
job that runs on every push, so the two smaller scenarios carry the regression
testing and this one is here to be run deliberately.

The mission
-----------

.. list-table::
   :header-rows: 0
   :widths: 30 70

   * - Agents
     - ``r1``, ``r2``, ``r3``, ``r4``
   * - The unknowns
     - whether each of north, south and east is blocked: three independent
       questions
   * - Depot rule
     - as before: a robot dispatched down a route stays on it
   * - Goal
     - twelve conjuncts: four robots × three routes

No robot can satisfy more than one of its own three conjuncts by driving, so
the fleet needs three survey trips and three broadcasts however it arranges
them.

Formalization
-------------

``robot-fleet-survey`` is the depot domain with a third route added and a
fourth agent in the problem. Nothing else changes, and the file says so: it is
generated from the same template, and the commentary explaining each construct
lives in the depot domain.

.. code-block:: text

   (:predicates
     (blocked-north) (blocked-south) (blocked-east)
     (at-north ?i - agent) (at-south ?i - agent) (at-east ?i - agent))

Three routes over four agents is what produces the size: 15 atoms (three route
states plus three positions per robot) and 48 grounded actions (four drive,
four look and eight report actions per route).

The initial theory leaves all three routes open, so the model designates eight
worlds, one per combination:

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

This task is not checked in. Ground it when you want to run it:

.. code-block:: bash

   ros2 run plansys2_epddl_grounder ground_epddl \
     -d <share>/plansys2_epddl_grounder/examples/robot-fleet-survey-domain.epddl \
     -p <share>/plansys2_epddl_grounder/examples/robot-fleet-survey-problem.epddl \
     -o /tmp/robot-fleet-survey.json

Grounding takes under a tenth of a second and reports the numbers above in the
task's ``planning-task-info``: 4 agents, 15 atoms, 48 actions, 8 initial
worlds. The action mapping for it *is* checked in, at
``plansys2_epistemic_planner/examples/mappings/robot-fleet-survey.json``, since
it is a fixed correspondence, not a derived artefact.

The policy
----------

AO* solves it at depth 9 and returns 28 nodes with eight leaves. Each survey
trip is three actions (drive, look, broadcast) and the look in the middle
of each is where the policy branches.

.. graphviz::
   :caption: The sensing skeleton of the survey policy. Each edge also carries the drive that precedes the look and the broadcast that follows it; each leaf is labelled with the three route states it corresponds to (B blocked, C clear) in the order the policy senses them: east, north, south.

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

The order the search settles on is east, then north, then south, with ``r1``,
``r2`` and ``r3`` taking one route each. ``r4`` never acts. It is a full member
of the fleet and four of the twelve goal conjuncts are about what it knows, and
it satisfies all four by listening, which is the clearest statement the
family makes of why the announcements are in the domain at all.

Running it
----------

As in the other two, with the survey task and mapping:

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

``plan_solver_timeout`` is the parameter to notice. Its default is 15 seconds,
which this scenario exceeds by a factor of four: leave it and the planner
returns no plan, with the search abandoned and not failed. The two smaller
scenarios never approach it, so it is easy to forget.

Performance
-----------

Measured on a release build with ``aostar`` and the heuristic the selection
policy chooses (``ks``):

.. list-table::
   :header-rows: 1
   :widths: 34 22 44

   * - Quantity
     - Value
     - Note
   * - Solution depth
     - 9
     - three drives, three looks, three broadcasts
   * - Nodes expanded
     - 5 029 063
     - three orders of magnitude above the depot
   * - Nodes generated
     - 5 746 557
     -
   * - Policy nodes
     - 28
     - seven sensing nodes and eight leaves
   * - Leaves
     - 8
     - one per combination of the three routes
   * - Wall clock
     - 57 s
     - search, this time; start-up is noise at this size
   * - Peak resident memory
     - 28 MB
     - the same as the corridor scenario, which finishes in 17 expansions

That last comparison is the point of running this scenario at all. Across the
three sizes the search grows by five orders of magnitude and the resident set
does not move, because what the planner holds is a bisimulation-contracted
model of what the agents know, not an enumeration of the histories that could
have produced it.

.. list-table:: The family, side by side
   :header-rows: 1
   :widths: 26 14 14 14 16 16

   * - Scenario
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

Each route added doubles the worlds and adds three to the solution depth, and
depth is what the search pays for exponentially: 17 expansions, then 1 670,
then five million. A fourth route would not be a fourth step of the same size.
