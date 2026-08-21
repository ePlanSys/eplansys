A First Plan
============

This page runs the epistemic solver once, on a task that ships with the
repository, and shows what it returns. It assumes a workspace built and
sourced as described in :doc:`installation`.

The problem
-----------

The problem is stated in EPDDL: a domain and a problem file, the epistemic
counterparts of a PDDL domain and problem. The smallest example ships with
``plansys2_epddl_grounder``:

.. code-block:: text

   <share>/plansys2_epddl_grounder/examples/muddy-children-domain.epddl
   <share>/plansys2_epddl_grounder/examples/muddy-children-problem.epddl

The solver does not search over EPDDL directly. It grounds the sources first —
building the initial Kripke model from the problem's finitary S5 theory and
instantiating each action's event model over the agents — into the IePC JSON
format it does search over. That is ``plank``'s work, run as a subprocess by
``plansys2_epddl_grounder``; see :doc:`../concepts/epddl` for the format and
:doc:`installation` for where the binary comes from.

Grounding happens on first use and again whenever the sources change, so
editing a domain and re-planning is enough. It can also be done by hand, which
is the way to inspect the task or to check it into a test:

.. code-block:: bash

   ros2 run plansys2_epddl_grounder ground_epddl \
     -d <share>/plansys2_epddl_grounder/examples/muddy-children-domain.epddl \
     -p <share>/plansys2_epddl_grounder/examples/muddy-children-problem.epddl \
     -o /tmp/muddy-children.json

Several already grounded tasks are checked into the planner package for its
tests, and can be used directly through ``task_file``:

.. code-block:: text

   plansys2_epistemic_planner/test/tasks/muddy-children-2.json
   plansys2_epistemic_planner/test/tasks/muddy-children-3.json
   plansys2_epistemic_planner/test/tasks/coin-in-the-box.json
   plansys2_epistemic_planner/test/tasks/coin-in-the-box-multipointed.json
   plansys2_epistemic_planner/test/tasks/active-muddy-child.json

``muddy-children-2.json`` is what grounding the example above produces.

Parameters
----------

``plansys2_bringup`` ships a parameters file that selects the epistemic solver,
``params/plansys2_epistemic_params.yaml``. Its planner section is:

.. code-block:: yaml

   planner:
     ros__parameters:
       plan_solver_timeout: 15.0
       plan_solver_plugins: ["EPISTEMIC"]
       EPISTEMIC:
         plugin: "plansys2/EpistemicPlanSolver"
         epddl_domain: ""
         epddl_problem: ""
         plank_command: ""
         task_file: ""
         heuristic: ""
         strategy: ""
         policy_file: ""
         action_mapping: ""
         conditional_plan: "policy"

The default parameters file keeps POPF, so this one has to be passed
explicitly. Every parameter is described in :doc:`../concepts/architecture`.

Bringing up the system
----------------------

.. code-block:: bash

   ros2 launch plansys2_bringup plansys2_bringup_launch_monolithic.py \
     model_file:=<domain.pddl> \
     params_file:=<share>/plansys2_bringup/params/plansys2_epistemic_params.yaml

``model_file`` is required by the launch file and feeds the domain expert. The
epistemic solver ignores it, but the executor does not: it is where the
behavior tree that drives the hardware for each action is found. The PDDL and
the EPDDL describe the same mission from two sides.

Executing a policy needs a fifth node: the behavior tree the epistemic builder
produces guards each action on a knowledge formula and updates the model
afterwards, and both are questions asked of the epistemic state. Add
``epistemic_state:=True`` and it is started and brought up with the other four,
by either launch file:

.. code-block:: bash

   ros2 launch plansys2_bringup plansys2_bringup_launch_monolithic.py \
     model_file:=<domain.pddl> \
     params_file:=<share>/plansys2_bringup/params/plansys2_epistemic_params.yaml \
     epistemic_state:=True

Give it the same problem the solver has, under ``epistemic_state:`` in the
parameters file:

.. code-block:: yaml

   epistemic_state:
     ros__parameters:
       epddl_domain: "/abs/path/to/domain.epddl"
       epddl_problem: "/abs/path/to/problem.epddl"

Naming the same two files twice is deliberate. The state grounds them itself
rather than being handed the planner's task, which would make the two nodes
depend on each other's start-up order; what has to agree is the problem, and
that is what the parameters file states.

The node runs as its own process even under the monolithic launch, and
``plansys2_node`` manages its lifecycle by name over services. That is what
keeps ``plansys2_bringup`` from linking against the epistemic packages, so the
package a classical PlanSys2 user builds is unchanged.

Running the node alone still works, and is the quickest way to inspect a model
without a planner:

.. code-block:: bash

   ros2 run plansys2_epistemic_executor epistemic_state_node --ros-args \
     -p epddl_domain:=<the same domain.epddl> \
     -p epddl_problem:=<the same problem.epddl>

Requesting a plan
-----------------

The solver takes the task from the first of three sources that is available:

#. the ``problem`` string of the ``planner/get_plan`` service, when that string
   is itself a grounded task JSON, recognised by its ``planning-task-info``
   key. A task in the request describes one call and outranks anything
   configured;
#. the ``epddl_domain`` and ``epddl_problem`` parameters, ground as above;
#. the ``task_file`` parameter.

Setting both EPDDL sources and a ``task_file`` grounds the sources and warns:
two descriptions of one problem drift apart, so name only one.

If none is supplied, the request fails with an explicit message rather than
attempting a translation from PDDL, which could not express event models or
per-agent observability anyway.

.. todo::

   The exact ``ros2 service call`` invocation for ``planner/get_plan``,
   including the service type and the shape of its request, is a
   ``plansys2_planner`` interface and is not documented in this repository.
   Fill in once the corresponding PlanSys2 page is cited or the call is
   verified against a running system.

What comes back
---------------

The solver searches for a policy and then validates it. What it returns depends
on the ``conditional_plan`` parameter:

``policy``
   The branches are kept, carried in the epistemic fields of each
   ``plansys2_msgs/PlanItem``. Running such a plan requires an executor
   configured with the epistemic behavior tree builder. This is the default in
   the shipped parameters file.

``flatten``
   Only the lowest-event branch is returned, with a warning. The result is a
   plain sequence that a stock PlanSys2 executor can run, and it is valid only
   if execution takes that one contingency.

``reject``
   A branching solution is refused rather than returned as a plan that is only
   conditionally valid.

A solution that does not branch is returned identically in all three modes, and
renders as the same flat behavior tree PlanSys2 would have built.

Action names
------------

The names the planner searches over are not PlanSys2 action expressions. A
grounded epistemic action is a single token such as ``ask_c1`` or
``pickup-A-hold_r2``, while the executor expects ``(ask c1)``: a name and its
parameters, looked up in the PDDL domain to find the behavior tree that drives
the hardware. The ``action_mapping`` parameter names a JSON file stating the
correspondence:

.. code-block:: json

   {
     "ask_c1":          "(ask c1)",
     "move-kitchen_r1": {"action": "(move r1 corridor kitchen)", "duration": 12.5}
   }

Duration is in seconds, defaults to 1.0, and must be positive. It is a property
of the robot's action implementation rather than of the plan, since the
epistemic planner is untimed, and it is what the executor schedules and times
out against.

With ``action_mapping`` unset, the solver falls back to a naming convention
that splits the grounded name at its first underscore and reads the rest as
parameters, keeping hyphens in the name: ``ask_c1`` becomes ``(ask c1)`` and
``pickup-A-hold_r2`` becomes ``(pickup-A-hold r2)``. That convention cannot
recover the parameter order the PDDL domain declares, so a mapping file should
be written before dispatching to real actions. An action the map does not cover
fails the planning request rather than reaching the executor as a name it
cannot dispatch.
