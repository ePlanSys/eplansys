ePlanSys
========

ePlanSys is an epistemic planning system for ROS 2. It extends `PlanSys2
<https://github.com/PlanSys2/ros2_planning_system>`_ with planning and
execution for agents that do not know everything about their environment:
domains where an action's precondition may be a statement about what an agent
knows, and where what to do next depends on what a sensing action turned out to
observe.

A classical PlanSys2 plan is a sequence. Every action's preconditions are known
to hold when its turn comes, so there is nothing to decide at run time. Under
partial observability that no longer holds, and the solution to a planning
problem is a policy: a tree whose branches are the possible observations. The
packages documented here add the pieces that produce such a policy, execute it,
and keep a model of what the agents know while it runs.

The classical PlanSys2 packages are included here with their upstream commit
history, and their documentation is the `PlanSys2 web page
<https://plansys2.github.io>`_. All of them have been modified: the extension
points the epistemic packages attach to were widened, compatibility shims were
added for Humble and Rolling, and defects found while running the system were
corrected. ``NOTICE`` records the package origins and the categories of
modification.

Packages
--------

.. list-table::
   :header-rows: 1
   :widths: 30 70

   * - Package
     - Contents
   * - ``plansys2_epistemic_planner``
     - The ``plansys2/EpistemicPlanSolver`` plan solver plugin: the Kripke
       state representation, DEL product update, bisimulation contraction,
       heuristics, search strategies, and the selection policy that picks
       between them.
   * - ``plansys2_epddl_grounder``
     - The EPDDL front end: it runs the ``plank`` toolkit as a subprocess to
       turn a domain and problem pair into the grounded task the planner and
       the epistemic state search over, and caches the result.
   * - ``plansys2_epistemic_msgs``
     - The six service definitions of the epistemic state: ``LoadTask``,
       ``CheckFormula``, ``ApplyAction``, ``GetGoal``, ``SetGoal`` and
       ``Announce``.
   * - ``plansys2_epistemic_executor``
     - The policy representation, its rendering as a behavior tree, the four
       epistemic behavior tree nodes, and the ``epistemic_state`` node that
       holds the model those nodes consult.
   * - ``plansys2_epistemic_bt_builder``
     - The ``plansys2::EpistemicBTBuilder`` plugin for ``plansys2_executor``,
       kept in its own package because it is the only piece that depends on
       the executor.
   * - ``plansys2_epistemic_perception``
     - The ``epistemic_perception`` node: named regions of an occupancy grid,
       classified as clear, blocked or undecided, and reported to the
       epistemic state as the outcome of a sensing action or as a public
       announcement.
   * - ``plansys2_aletheia_plan_solver``
     - An alternative plan solver plugin that runs the same planner as an
       external process instead of in process, for deployments where the
       planner binary is built and versioned separately.
   * - ``plansys2_tui_cli``
     - The command line front end: the ``ros2 plansys2`` verbs, among them
       ``epistemic``, which is the terminal for what the agents know, and a
       four-quadrant Textual dashboard over the running system.
   * - ``eplansys``
     - A metapackage that installs PlanSys2 and all of the above as one unit.
       It builds nothing itself.

Contents
--------

.. toctree::
   :maxdepth: 2
   :caption: Getting started

   getting_started/installation
   getting_started/first_plan
   getting_started/command_line

.. toctree::
   :maxdepth: 2
   :caption: Concepts

   concepts/epistemic_models
   concepts/epddl
   concepts/architecture

.. toctree::
   :maxdepth: 2
   :caption: Tutorials

   tutorials/index

.. toctree::
   :maxdepth: 2
   :caption: Reports

   reports/epistemic_end_to_end

.. toctree::
   :maxdepth: 2
   :caption: About

   about/contributing
   about/license
