Fleet: the blocked corridor
===========================

Two survey robots share a depot. One corridor leads out of it, and it may be
blocked by debris. Only a robot standing at the junction can see down it, and a
robot watching another one look learns *that* it looked, not *what* it saw.
Both robots route around the corridor, so the mission is not finished until
both know whether it is passable.

Nothing in this scenario changes the corridor. What changes is who knows about
it, so a classical planner has nothing to work with here, and
the solution is a policy and not a sequence.

The smallest scenario of the fleet family, and the one to read first: one
corridor, one sensing action, one branch.

The mission
-----------

.. list-table::
   :header-rows: 0
   :widths: 30 70

   * - Agents
     - ``r1``, ``r2``: two survey robots, symmetric, either could do the job
   * - The unknown
     - whether the corridor is blocked; nobody has looked
   * - Actions
     - drive to the junction, look down the corridor, broadcast what was found
   * - Goal
     - both robots know *whether* the corridor is blocked

The goal is a *knowing-whether* goal, and deliberately so. A mission that only
succeeds when the corridor turns out to be blocked is not a mission, and a plan
for it would be unexecutable half the time.

Formalization
-------------

The domain is ``robot-fleet``, in
``plansys2_epddl_grounder/examples/robot-fleet-domain.epddl``. Two predicates
carry the whole scenario:

.. code-block:: text

   (:predicates
     (blocked)                    ; the corridor is obstructed
     (at-junction ?i - agent)     ; ?i is where the corridor can be seen
   )

The three mechanisms are one action each, and the difference between them is
entirely in who observes what.

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

``report-clear`` is the mirror of ``report-blocked``: a robot can only report
what it found, so the domain has two announcements in place of one action with
a parameter.

.. graphviz::
   :caption: What each action type does to the fleet's knowledge

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
       o_act -> o_all [label=" world changes,\l everyone sees it\l"];
     }

     subgraph cluster_sense {
       label="semi-private-sensing:  inspect r1";
       labelloc=t; fontsize=10; style=dashed; color="#aaaaaa"; margin=12;
       s_act [label="r1 looks", style="rounded,filled", fillcolor="#eef3fb"];
       s_r1  [label="r1 knows whether\nthe corridor is blocked"];
       s_r2  [label="r2 knows only\nthat r1 looked"];
       s_act -> s_r1 [label=" Fully"];
       s_act -> s_r2 [label=" Partially"];
     }

     subgraph cluster_say {
       label="public-announcement:  report-blocked r1";
       labelloc=t; fontsize=10; style=dashed; color="#aaaaaa"; margin=12;
       a_act [label="r1 broadcasts", style="rounded,filled", fillcolor="#eef3fb"];
       a_all [label="r1, r2 know\nthe corridor is blocked"];
       a_act -> a_all [label=" knowledge is factive,\l so hearing it is learning it\l"];
     }

     o_all -> s_act [style=invis];
     s_r2  -> a_act [style=invis];
   }

Semi-private sensing is what makes the fleet talk. If looking were public, one
robot looking would inform the other and no announcement would be needed; if it
were fully private, the other robot would not even know a look had happened.
A camera on one robot is neither: the fleet sees the behaviour, not the image.

The initial theory
~~~~~~~~~~~~~~~~~~

The problem file states what is common knowledge at the depot and, just as
importantly, what it leaves open:

.. code-block:: text

   (:agents r1 r2)

   (:init
     (:and
       (:forall (?i - agent) ([C. All] (not (at-junction ?i))))
       (:forall (?i - agent) ([C. All] (<Kw. ?i> (blocked))))))

   (:goal
     (and ([Kw. r1] (blocked)) ([Kw. r2] (blocked))))

It never says whether the corridor is blocked. The theory is therefore
satisfied by two worlds, both designated: the model is multi-pointed, and the
corridor is one way or the other without anyone (including the
planner) knowing which. That is the property that makes the solution branch,
and the reason this scenario exists.

.. graphviz::
   :caption: The model through one execution: sensing removes r1's uncertainty, the announcement removes r2's

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

A double circle is a designated world, one the fleet cannot rule out as the
actual one. An edge labelled ``r1`` is r1 being unable to tell the two apart.
The mission is finished when no agent has an edge left, the condition ``Kw``
expresses.

Grounding
---------

``plank`` turns the EPDDL pair into the IePC JSON the planner searches over:

.. code-block:: bash

   ros2 run plansys2_epddl_grounder ground_epddl \
     -d <share>/plansys2_epddl_grounder/examples/robot-fleet-domain.epddl \
     -p <share>/plansys2_epddl_grounder/examples/robot-fleet-problem.epddl \
     -o /tmp/robot-fleet.json

The result is the checked-in
``plansys2_epistemic_planner/test/tasks/robot-fleet.json``: 2 agents, 3 atoms,
8 grounded actions, and 2 initial worlds, both designated. Grounding also
happens automatically on first use when the solver is given ``epddl_domain``
and ``epddl_problem`` instead of a ``task_file``.

The policy
----------

AO* solves it at depth 3 and returns four policy nodes: drive out, look, and
one announcement per outcome.

.. graphviz::
   :caption: The corridor policy. The branch is at the sensing action; each leaf is one way the corridor can turn out.

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

Two things in that tree are worth reading carefully.

The **knowledge requirement** on each announcement, ``(K r1 blocked)`` and
``(K r1 (not blocked))``, is not a PDDL precondition and cannot be checked
against the problem expert, because no predicate records it. It is checked
against the epistemic state by ``CheckKnowledge``, and it is what stops the
fleet from broadcasting a guess.

The **branch** is the planner declining to commit. Both continuations are in
the tree; which one runs is decided while the robot is standing at the junction
with its camera on. A flattened version of this plan would be correct half the
time.

Note also what the planner worked out: both robots could have driven out and
looked, at four actions; one robot going, looking and broadcasting costs three.
It found the cheaper one, which is to say it worked out that talking beats
everybody going to see for themselves.

Running it
----------

**The planner.** Point the solver at the task and its action mapping, and ask
for the branches to be kept:

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

``conditional_plan: "policy"`` is the setting that matters. The default,
``"flatten"``, follows one branch and warns; it exists for a stock PlanSys2
executor, which can only run a sequence.

**The mapping.** The planner searches over grounded epistemic names and the
executor drives PDDL actions, so the two vocabularies are related by a file:

.. code-block:: json

   {
     "goto-junction_r1": {"action": "(goto_junction r1)", "duration": 30.0},
     "inspect_r1":       {"action": "(inspect_corridor r1)", "duration": 5.0},
     "report-blocked_r1": {"action": "(report_blocked r1)", "duration": 1.0},
     "report-clear_r1":   {"action": "(report_clear r1)", "duration": 1.0}
   }

The durations belong to the robot's implementation of each action, not to the
plan: the epistemic planner is untimed, and these are what the executor
schedules and times out against.

**The executor.** Two parameters select the epistemic path:

.. code-block:: yaml

   executor:
     ros__parameters:
       bt_builder_plugin: "EpistemicBTBuilder"
       bt_node_plugins: ["libplansys2_epistemic_bt_nodes.so"]

**The state.** ``epistemic_state`` must be loaded from the same task the
planner solved, or the policy's knowledge requirements would be checked in a
vocabulary it does not speak:

.. code-block:: bash

   ros2 run plansys2_epistemic_executor epistemic_state_node --ros-args \
     -p task_file:=<repo>/plansys2_epistemic_planner/test/tasks/robot-fleet.json

**The observation.** With two designated worlds the epistemic state cannot
determine which outcome the sensing produced, so the answer must come from the
robot that performed it. The ``inspect_corridor`` performer supplies it on
finishing, in the ``outcome`` field of ``ActionExecution``:

.. code-block:: cpp

   // in the performer's do_work, once the sensor has answered
   finish(true, 1.0, "Corridor inspected", clear ? "e-inspect-clear"
                                                 : "e-inspect-blocked");

The token must name an outcome of the action's event model, since the policy
branches on it. ``status`` remains the human-readable message. The packaged
action template then carries the value across without further configuration:
``ExecuteAction`` writes it to the blackboard and ``ApplyEpistemicUpdate``
reads it.

An observation originating elsewhere, such as an independently operating
perception node, is supplied by binding ``ApplyEpistemicUpdate``'s ``observed``
port in your own action template. See
:doc:`../reports/epistemic_end_to_end` for a worked binding.

Performance
-----------

Measured with the packaged grounded task, ``aostar`` with the heuristic the
selection policy chooses (``ks``), on a release build:

.. list-table::
   :header-rows: 1
   :widths: 34 22 44

   * - Quantity
     - Value
     - Note
   * - Solution depth
     - 3
     - drive, look, report
   * - Nodes expanded
     - 17
     - the search barely searches
   * - Nodes generated
     - 19
     -
   * - Policy nodes
     - 4
     - one per action, both announcements included
   * - Leaves
     - 2
     - one per way the corridor can turn out
   * - Wall clock
     - 0.5 s
     - dominated by process start-up, not by search
   * - Peak resident memory
     - 28 MB
     - essentially the ROS process itself

This scenario is small enough to run in a test suite, and it does:
``plansys2_tests/test_5`` executes it end to end over a live node graph, twice,
with the corridor blocked in one run and clear in the other.

Next
----

:doc:`fleet_depot` is the same three mechanisms with two corridors and three
robots, where the planner has to divide the labour and the policy branches
twice.
