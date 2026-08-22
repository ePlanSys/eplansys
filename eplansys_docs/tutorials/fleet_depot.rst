Fleet: the two-route depot
==========================

The corridor scenario one size up. Three robots, a depot with two ways out ---
north and south --- and no reason to think the state of one route says anything
about the other. Every robot has to know the state of both before the fleet
commits to a route.

What grows here is not only the numbers. With one corridor there was a single
thing to find out and any robot could go and find it. With two, and a depot
rule that a robot dispatched down a route stays on it, no single robot can
survey both: the mission has to be divided, and whoever stays behind has to be
told. The policy branches twice, and the second half of it has to be correct
inside either branch of the first.

The mission
-----------

.. list-table::
   :header-rows: 0
   :widths: 30 70

   * - Agents
     - ``r1``, ``r2``, ``r3``
   * - The unknowns
     - whether the north route is blocked; whether the south route is
   * - Depot rule
     - a robot at either junction cannot drive to the other one
   * - Goal
     - six conjuncts: each of the three robots knows whether each route is
       blocked

Two of those six conjuncts is the most any robot can satisfy by itself, and
only by driving somewhere and looking. The other four have to arrive by radio.

Formalization
-------------

The domain, ``robot-fleet-depot``, is the corridor domain with each mechanism
duplicated per route, and with the driving preconditions carrying the depot
rule:

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

The precondition is what forbids driving from one junction to the other, and it
is a depot rule rather than a modelling shortcut: it is what makes this a fleet
problem instead of one robot's tour.

Sensing and announcing are unchanged, twice over --- ``inspect-north`` and
``inspect-south`` are semi-private, the four ``report-*`` actions are public
announcements guarded by the speaker actually knowing.

The initial theory leaves both routes open:

.. code-block:: text

   (:agents r1 r2 r3)

   (:init
     (:and
       (:forall (?i - agent) ([C. All] (and (not (at-north ?i)) (not (at-south ?i)))))
       (:forall (?i - agent) ([C. All] (<Kw. ?i> (blocked-north))))
       (:forall (?i - agent) ([C. All] (<Kw. ?i> (blocked-south))))))

Two independent unknowns give four worlds, all designated: blocked/blocked,
blocked/clear, clear/blocked, clear/clear. The fleet cannot tell any of them
apart to begin with.

.. graphviz::
   :caption: The four initial worlds. Every robot's accessibility relation is the whole square, so nobody knows anything about either route.

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

The result is the checked-in
``plansys2_epistemic_planner/test/tasks/robot-fleet-depot.json``: 3 agents,
8 atoms, 24 grounded actions, 4 initial worlds, all designated.

The policy
----------

AO* solves it at depth 6 and returns twelve nodes. Read down the left edge:
``r1`` drives north, looks, and broadcasts; then ``r2`` drives south, looks, and
broadcasts. The whole of that second half appears twice, once inside each
outcome of the first look --- not because the plan repeats itself, but because
those are different plans: they run against different epistemic states, and the
announcement that ends each one differs.

.. graphviz::
   :caption: The depot policy. Sensing nodes are shaded; each of the four leaves is one way the depot can turn out.

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

The division of labour is the planner's choice, not the domain's. One robot
walking both routes is impossible here, but ``r1`` and ``r3`` splitting them
would cost exactly what ``r1`` and ``r2`` splitting them costs; the search has
to consider both and settles on one. Six actions on every path: three per
route, and never a robot going to see something it has already been told.

Running it
----------

Identical to :doc:`fleet_corridor` with the depot task and mapping
substituted:

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

The mapping's PDDL side takes a route parameter, since one PDDL action covers
both routes where the epistemic domain has one grounded action per route:

.. code-block:: json

   {
     "goto-north_r1":  {"action": "(goto_junction r1 north)", "duration": 30.0},
     "inspect-south_r2": {"action": "(inspect_corridor r2 south)", "duration": 5.0},
     "report-south-clear_r2": {"action": "(report_clear r2 south)", "duration": 1.0}
   }

Observations are reported per action, not per mission: the depot has two things
to find out and they need not agree, so whatever binds the ``observed`` port
has to answer separately for ``(inspect_corridor r1 north)`` and
``(inspect_corridor r2 south)``.

Performance
-----------

.. list-table::
   :header-rows: 1
   :widths: 34 22 44

   * - Quantity
     - Value
     - Note
   * - Solution depth
     - 6
     - two survey trips, two broadcasts, two drives
   * - Nodes expanded
     - 1 670
     - two orders of magnitude above the corridor
   * - Nodes generated
     - 2 010
     -
   * - Policy nodes
     - 12
     - the second half appears once per first-look outcome
   * - Leaves
     - 4
     - one per combination of the two routes
   * - Wall clock
     - 0.5 s
     - still dominated by process start-up
   * - Peak resident memory
     - 28 MB
     - unchanged from the corridor: the model does not grow with the search

The last row is the interesting one. Between the corridor and the depot the
search grows by a factor of a hundred and the memory does not move, because
what is stored is a bisimulation-contracted model rather than an enumeration of
histories.

``plansys2_tests/test_6`` runs this scenario end to end, twice, with the routes
in opposite states --- which is the case that exercises a switch nested inside
another switch's continuation.

Next
----

:doc:`fleet_survey` is the same domain at four robots and three routes, where
the search grows by another three orders of magnitude and the memory still does
not.
