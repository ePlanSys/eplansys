.. meta::
   :description: ePlanSys is an epistemic planning system for ROS 2. It extends
      PlanSys2 with EPDDL, dynamic epistemic logic, and execution of branching
      policies for robots acting under partial observability.
   :keywords: ePlanSys, epistemic planning, EPDDL, PlanSys2, ROS 2, dynamic
      epistemic logic, DEL, partial observability, multi-robot

ePlanSys
========

**Epistemic planning and policy execution for ROS 2**

Release |release| · Apache License 2.0 ·
`source <https://github.com/ePlanSys/eplansys>`_ ·
`releases <https://github.com/ePlanSys/eplansys/releases>`_ ·
ROS 2 Humble and Rolling

.. raw:: html

   <figure class="tutorial-video">
     <video controls preload="metadata" playsinline
            poster="_static/video/eplansys-tutorial-poster.png">
       <source src="_static/video/eplansys-tutorial.mp4" type="video/mp4">
       <a href="_static/video/eplansys-tutorial.mp4">Download the video (MP4, 66 s)</a>
     </video>
     <figcaption>
       Worked example (66 s, no audio): the site-survey mission of
       <code>eplansys_demo</code>. A scout must learn whether a site is
       contaminated and tell a teammate, while an observer must not learn it.
       The planner chooses a private message over a broadcast because a
       broadcast falsifies the secrecy conjunct. The Kripke models are
       computed from the grounded task by the DEL product update; the planner
       output is from the pinned planner revision.
     </figcaption>
   </figure>

Abstract
--------

ePlanSys is a ROS 2 system for planning over Dynamic Epistemic Logic (DEL)
models :cite:p:`baltag1998logic,vanditmarsch2007del` and executing the
resulting branching policies through PlanSys2 :cite:p:`martin2021plansys2`.
Tasks are stated in EPDDL :cite:p:`epddl2026guideline`, grounded by plank
:cite:p:`plank`, and solved by an AND/OR search over multi-pointed Kripke
models :cite:p:`aletheia`. Each action of a policy is rendered as a behavior
tree :cite:p:`colledanchise2018bt` in which the PlanSys2 action subtree is
preceded by a guard on the action's knowledge precondition and followed by the
DEL product update with the outcome the robot observed and a branch on that
outcome. A run-time epistemic state node tracks the team's Kripke model,
repairs it when an observation contradicts it, and supplies the reached belief
as the initial state of the next planning request. All components attach to
PlanSys2 through its plugin interfaces; a classical plan renders to the same
behavior tree as in upstream PlanSys2.

Problem setting
---------------

An epistemic planning task :cite:p:`bolander2011epistemic` is a triple
:math:`T = (s_0, A, \gamma)` where

* :math:`s_0 = (M, W^*)` is a multi-pointed Kripke model over a set of agents
  :math:`Ag` and atoms :math:`P`, with designated worlds :math:`W^*`;
* :math:`A` is a finite set of actions, each an event model whose events carry
  a precondition, a postcondition, and per-agent observability;
* :math:`\gamma` is a formula of the multi-agent epistemic language
  (:math:`K_i`, :math:`B_i`, :math:`\mathit{Kw}_i`, :math:`C_G`).

Applying action :math:`a` in state :math:`s` yields the product update
:math:`s \otimes a`. When :math:`a` has several designated events and the
acting agent can distinguish them, :math:`s \otimes a` splits into one state
per observable outcome. A solution is therefore a *policy*: a finite tree whose
nodes are actions and whose arcs are outcomes, such that every action is
applicable in the state its node reaches and every leaf state satisfies
:math:`\gamma`. A classical plan is the special case of a policy without
branches.

PlanSys2 and ROSPlan :cite:p:`cashmore2015rosplan` represent the world as a set
of PDDL facts :cite:p:`fox2003pddl21` and execute sequences. They cannot
represent statements such as :math:`K_{r_1} \neg K_{r_2}\, \mathit{blocked}`,
distinguish an action by who observes it, or branch on a sensing outcome.
Contingent and POMDP planners
:cite:p:`bonet2000planning,hoffmann2005contingent,kaelbling1998pomdp` branch on
observations but model a single agent's uncertainty. DEL planners
:cite:p:`le2018efp,fabiano2020efp2` model higher-order knowledge but stop at the
plan. ePlanSys connects the second class to ROS 2 execution.

Components
----------

.. list-table::
   :header-rows: 1
   :widths: 30 70

   * - Package
     - Contents
   * - ``plansys2_epistemic_planner``
     - ``plansys2/EpistemicPlanSolver``, a plan solver plugin linking the
       Aletheia planning core: Kripke state representation, DEL product
       update, bisimulation contraction, heuristics, search strategies and
       strategy selection (:doc:`concepts/epistemic_models`).
   * - ``plansys2_epddl_grounder``
     - EPDDL front end. Runs ``plank`` as a subprocess to ground a domain and
       problem into the task format the planner and the state read, and caches
       the result (:doc:`concepts/epddl`).
   * - ``plansys2_epistemic_msgs``
     - Nine service definitions of the epistemic state: ``LoadTask``,
       ``CheckFormula``, ``ApplyAction``, ``GetGoal``, ``SetGoal``,
       ``Announce``, ``GetAgentPerspective``, ``GetEpistemicDomain`` and
       ``GetEpistemicActionDetails``.
   * - ``plansys2_epistemic_executor``
     - Policy representation and validation, rendering of a policy as a
       behavior tree, the five epistemic behavior tree nodes, and the
       ``epistemic_state`` lifecycle node.
   * - ``plansys2_epistemic_bt_builder``
     - ``plansys2::EpistemicBTBuilder``, the behavior tree builder plugin for
       ``plansys2_executor``. The only epistemic package that depends on the
       executor.
   * - ``plansys2_epistemic_perception``
     - ``epistemic_perception`` lifecycle node. Classifies named regions of an
       occupancy grid as clear, blocked or unknown and reports them to the
       epistemic state as sensing outcomes or public announcements.
   * - ``plansys2_aletheia_plan_solver``
     - ``plansys2/AletheiaPlanSolver``, a plan solver plugin that runs the
       planner binary as a subprocess.
   * - ``plansys2_tui_cli``
     - ``ros2 plansys2`` verbs, including ``epistemic``, and a Textual
       dashboard (:doc:`getting_started/command_line`).
   * - ``eplansys_demo``
     - Example missions; the corridor mission runs without a simulator
       (:doc:`demos/index`).
   * - ``eplansys``
     - Metapackage depending on PlanSys2 and all of the above.

The classical PlanSys2 packages are included with their upstream commit
history; their documentation is the `PlanSys2 web page
<https://plansys2.github.io>`_. They carry modifications in four categories,
recorded in ``NOTICE``: widened extension points, bring-up of the epistemic
nodes, Humble/Rolling compatibility shims, and defect fixes
(:doc:`about/license`).

Citing
------

.. code-block:: bibtex

   @software{eplansys,
     author  = {Vasquez Morales, Haniel Ulises},
     title   = {{ePlanSys}: Epistemic Planning and Execution for {ROS}~2},
     url     = {https://github.com/ePlanSys/eplansys},
     year    = {2026}
   }

Contents
--------

.. toctree::
   :maxdepth: 2
   :numbered:

   getting_started/index
   concepts/index
   demos/index
   tutorials/index
   execution/index
   reports/epistemic_end_to_end

.. toctree::
   :maxdepth: 1
   :caption: Back matter

   references
   about/contributing
   about/license
