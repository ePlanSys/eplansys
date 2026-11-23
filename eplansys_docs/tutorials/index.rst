Tutorials
=========

Three instances of one domain family, of increasing size. A fleet of survey
robots shares a depot; the routes out of it may be blocked. Only a robot at a
junction can observe a route, and a robot watching another inspect a route
learns that the inspection happened but not its result. The goal is that every
robot knows whether each route is passable before the fleet commits to one.

No action changes whether a route is blocked; actions change which robots know
it. The initial state leaves every route's status open, so solutions sense and
branch on the result.

.. list-table:: Tutorial instances.
   :header-rows: 1
   :widths: 30 12 12 12 34

   * - Scenario
     - Robots
     - Routes
     - Worlds
     - Feature
   * - :doc:`fleet_corridor`
     - 2
     - 1
     - 2
     - One sensing action, one branch
   * - :doc:`fleet_depot`
     - 3
     - 2
     - 4
     - Division of labour; nested branching
   * - :doc:`fleet_survey`
     - 4
     - 3
     - 8
     - Scale (five million expansions); a robot that only listens

The corridor chapter introduces every construct used by the other two; the
depot and survey chapters describe only the differences and report how the
search grows.

The corridor and depot instances are executed end to end by ``plansys2_tests``
over a live ROS graph, in each possible world state
(:doc:`../reports/epistemic_end_to_end`). The survey instance is too slow for
per-commit CI and is run manually.

The EPDDL sources are in ``plansys2_epddl_grounder/examples``; the two smaller
instances are also checked in grounded under
``plansys2_epistemic_planner/test/tasks`` and run without plank. A workspace
built as in :doc:`../getting_started/installation` is assumed.

The classical PlanSys2 tutorials (terminal, domain and problem experts, action
performers) are on the `PlanSys2 web page <https://plansys2.github.io>`_ and
apply unchanged.

.. toctree::
   :maxdepth: 1

   fleet_corridor
   fleet_depot
   fleet_survey
