The corridor
============

Two robots and a corridor of unknown state. The goal is
:math:`\mathit{Kw}_{r_1}\,\mathit{blocked} \wedge \mathit{Kw}_{r_2}\,
\mathit{blocked}`. The goal concerns knowledge of the corridor and is
independent of its actual state, so the solution is a policy: ``r1`` drives to
the junction, inspects, and announces the result, the announcement being
selected at run time. The task is the one of :doc:`../tutorials/fleet_corridor`.

The EPDDL sources are grounded by ``plank`` at start-up
(:doc:`../getting_started/installation`).

Usage
-----

.. code-block:: bash

   ros2 launch eplansys_demo corridor_launch.py
   ros2 launch eplansys_demo corridor_launch.py corridor:=blocked

``corridor`` sets the outcome the inspection performer reports. The planner
does not receive it; the policy is identical in both runs, and the executed
branch differs.

Data flow
---------

.. _fig-corridor-demo:

.. graphviz::
   :caption: Data flow of the corridor mission. Bold edges are the additions
      to a classical PlanSys2 deployment.

   digraph corridor {
     rankdir=LR;
     nodesep=0.5;
     ranksep=0.7;
     node [shape=box, fontname="sans-serif", fontsize=10, margin="0.14,0.10"];
     edge [fontname="sans-serif", fontsize=9];

     task      [label="EPDDL domain and problem\ntwo designated worlds"];
     planner   [label="planner"];
     policy    [label="policy\nfour nodes, branching", shape=note];
     executor  [label="executor"];
     performer [label="performer"];
     state     [label="epistemic state"];

     task      -> planner   [label="ground (plank)"];
     planner   -> policy;
     policy    -> executor;
     executor  -> performer [label="dispatch"];

     performer -> state     [label="  observed outcome", penwidth=1.8];
     state     -> executor  [label="  branch", penwidth=1.8,
                             constraint=false];
   }

Log
---

.. code-block:: text

   [planner]         [epistemic] policy with 4 nodes, branching
   [epistemic_state] applied goto-junction_r1: 2 worlds, 2 designated
   [corridor_actions] inspect_corridor: observed e-inspect-clear
   [epistemic_state] applied inspect_r1 -> e-inspect-clear: 2 worlds, 1 designated
   [epistemic_bt]    [goal] (and (Kw r1 blocked) (Kw r2 blocked)) holds

.. list-table:: Interpretation of the log.
   :header-rows: 1
   :widths: 38 62

   * - Line
     - Meaning
   * - ``policy with 4 nodes, branching``
     - Three actions execute; the fourth node is the announcement for the
       outcome that does not occur.
   * - ``goto-junction_r1: 2 worlds, 2 designated``
     - The ontic action changes the robot's position. Both corridor states
       remain designated.
   * - ``observed e-inspect-clear``
     - The performer's report, passed as the outcome argument of ``finish``.
   * - ``inspect_r1 -> e-inspect-clear: 2 worlds, 1 designated``
     - The inspection eliminates one world from the designated set. Both
       worlds remain in the model, since ``r2`` cannot yet distinguish them.
   * - ``[goal] ... holds``
     - ``CheckEpistemicGoal`` evaluates the goal in the resulting model.
       Reaching a leaf does not by itself establish the goal.

With ``corridor:=blocked`` the third line reports ``e-inspect-blocked``, and
``report_blocked`` executes in place of ``report_clear``.

Source files
------------

``corridor_actions.cpp``
   Four performers that wait for their duration and finish. Only
   ``inspect_corridor`` differs from a classical performer: it passes the
   observed outcome to ``finish``.

``corridor_mission.cpp``
   Declares the instances and initial predicates in the problem expert, sets
   the PDDL goal, and runs a ``MissionSupervisor``
   (:ref:`arch-supervisor`), which plans, executes, and replans on failure.

The EPDDL domain and problem, the PDDL domain, and the action mapping are the
remaining inputs; :doc:`../getting_started/first_plan` describes each.
:doc:`survey` extends the mission to three agents and a negative goal.
