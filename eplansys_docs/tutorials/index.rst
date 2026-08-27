Tutorials
=========

Three robotics scenarios, one domain family, three sizes. A fleet of survey
robots shares a depot; the routes out of it may be blocked by debris; only a
robot standing at a junction can see down a route, and a robot watching another
one look learns *that* it looked, not *what* it saw. Before the fleet commits
to a route, every robot has to know which routes are passable.

That is a planning problem no classical planner can state. Nothing an action
does here changes whether a corridor is blocked. What changes is who knows
about it, and the state of the corridor is not in the initial situation to
begin with, so the solution cannot be a sequence. It has to be a policy that
senses and then branches on what was sensed.

.. list-table::
   :header-rows: 1
   :widths: 30 12 12 12 34

   * - Scenario
     - Robots
     - Routes
     - Worlds
     - What it adds
   * - :doc:`fleet_corridor`
     - 2
     - 1
     - 2
     - The mechanism at its smallest: one sensing action, one branch
   * - :doc:`fleet_depot`
     - 3
     - 2
     - 4
     - Division of labour, and a branch inside a branch
   * - :doc:`fleet_survey`
     - 4
     - 3
     - 8
     - Scale: five million expansions, and a robot that only listens

Read them in order. The corridor scenario introduces every construct the other
two use; the depot and survey pages describe only what is new, and report what
grows.

The first two are executed end to end by the test suite, over a live ROS graph
with the world in each of its possible states; see
:doc:`../reports/epistemic_end_to_end`. The third is too slow for a suite that
runs on every push, and is there to be run by hand.

All three ship as EPDDL sources in ``plansys2_epddl_grounder/examples``, with
the two smaller ones also checked in already grounded under
``plansys2_epistemic_planner/test/tasks``, so they can be run without an EPDDL
toolchain. They assume a workspace built and sourced as described in
:doc:`../getting_started/installation`.

The classical PlanSys2 tutorials (the terminal, the domain and problem
experts, writing action performers) are on the `PlanSys2 web page
<https://plansys2.github.io>`_ and apply unchanged here.

.. toctree::
   :maxdepth: 1

   fleet_corridor
   fleet_depot
   fleet_survey
