The corridor
============

Two robots and a corridor nobody has looked down. The goal is that both come to
know whether it is blocked.

No classical planner can state that goal. There is no fact about the corridor to
plan towards --- the corridor is blocked or it is not, and the plan is the same
either way --- only a state of knowledge to reach. The solution is therefore a
policy and not a sequence: ``r1`` drives out, looks, and reports what it found,
and which report it makes is decided while the mission runs.

The demo needs no simulator and no hardware. What it shows is the planning, and
everything that would otherwise stand between a reader and that is left out.

Running it
----------

.. code-block:: bash

   ros2 launch eplansys_demo corridor_launch.py
   ros2 launch eplansys_demo corridor_launch.py corridor:=blocked

Run it both ways. The ``corridor`` argument is what the robot turns out to see
when it looks; it is not given to the planner, and the plan does not depend on
it. The execution does.

What runs
---------

.. graphviz::
   :caption: The corridor mission, from a grounded task to a branch taken

   digraph corridor {
     rankdir=LR;
     nodesep=0.5;
     ranksep=0.7;
     node [shape=box, fontname="sans-serif", fontsize=10, margin="0.14,0.10"];
     edge [fontname="sans-serif", fontsize=9];

     task      [label="grounded task\ntwo worlds, both possible"];
     planner   [label="planner"];
     policy    [label="policy\nfour nodes, branching", shape=note];
     executor  [label="executor"];
     performer [label="performer"];
     state     [label="epistemic state"];

     task      -> planner   [label="EPDDL"];
     planner   -> policy;
     policy    -> executor;
     executor  -> performer [label="dispatch"];

     performer -> state     [label="  outcome observed", penwidth=1.8];
     state     -> executor  [label="  branch to take", penwidth=1.8,
                             constraint=false];
   }

The two bold edges are the mission's whole difficulty. Everything else a
classical PlanSys2 deployment already does.

What to watch
-------------

The mission prints its own progress, and four lines carry the argument.

.. code-block:: text

   [planner]         [epistemic] policy with 4 nodes, branching
   [epistemic_state] applied goto-junction_r1: 2 worlds, 2 designated
   [corridor_actions] inspect_corridor: observed e-inspect-clear
   [epistemic_state] applied inspect_r1 -> e-inspect-clear: 2 worlds, 1 designated
   [epistemic_bt]    [goal] (and (Kw r1 blocked) (Kw r2 blocked)) holds

Read in order:

``policy with 4 nodes, branching``
   Four nodes for a mission that executes three actions. The fourth is the
   branch not taken, and it is in the plan because the planner could not know
   which one would be.

``2 worlds, 2 designated``
   After driving out, the model still holds both possibilities: the corridor is
   blocked in one world and clear in the other, and the robot cannot yet tell
   which it is in. Driving changed where the robot is and nothing about what it
   knows.

``observed e-inspect-clear``
   The performer reporting what it saw. This is the one thing a sensing action
   has to do that an ordinary one does not, and it is one argument on
   ``finish``. Everything after it is the framework's business.

``2 worlds, 1 designated``
   The looking has ruled a world out. The model still contains both, because
   both remain describable; only one is still held to be actual.

``(Kw r1 blocked) ... holds``
   The epistemic goal, checked against the model that resulted rather than the
   one planning assumed. Reaching a leaf of a policy is not the same as having
   achieved the goal.

Running with ``corridor:=blocked`` changes the third line, and with it the
branch: ``report_blocked`` runs and ``report_clear`` does not. The policy is
identical in both runs.

What the demo supplies
----------------------

Two files, and neither is about knowledge:

``corridor_actions.cpp``
   Four performers that wait and then finish. They stand in for hardware, and a
   deployment would replace the waiting rather than the structure. Only one of
   them differs from a classical performer: ``inspect_corridor`` names the
   outcome it observed when it calls ``finish``.

``corridor_mission.cpp``
   The three steps anyone would otherwise type into ``ros2 plansys2 terminal``:
   say what exists, name the goal, run. Doing it from a node is what makes the
   demo one command.

Nothing else in the demo is code. The task, the PDDL and the mapping are the
three files a mission of your own would also need, and
:doc:`../getting_started/first_plan` says what each is for.
