A First Plan
============

This page runs the epistemic solver once, on a task that ships with the
repository, and shows what it returns. It assumes a workspace built and
sourced as described in :doc:`installation`.

The task
--------

The solver reads a grounded epistemic planning task in the IePC JSON format.
It does not read EPDDL, and it does not translate the PDDL problem the domain
expert holds; see :doc:`../concepts/epddl` for why. Several grounded tasks are
checked into the planner package for its tests:

.. code-block:: text

   plansys2_epistemic_planner/test/tasks/muddy-children-2.json
   plansys2_epistemic_planner/test/tasks/muddy-children-3.json
   plansys2_epistemic_planner/test/tasks/coin-in-the-box.json
   plansys2_epistemic_planner/test/tasks/coin-in-the-box-multipointed.json
   plansys2_epistemic_planner/test/tasks/active-muddy-child.json

``muddy-children-2.json`` is the smallest solvable one and is used below.

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
epistemic solver ignores it: the grounded task is self-contained.

Executing a policy also needs the epistemic state, which is a separate node and
must hold the same task:

.. code-block:: bash

   ros2 run plansys2_epistemic_executor epistemic_state_node \
     --ros-args -p task_file:=<the same grounded task JSON>

Requesting a plan
-----------------

The solver takes the task from whichever of two sources arrives first: the
``problem`` string of the ``planner/get_plan`` service, when that string is
itself a grounded task JSON, or the ``task_file`` parameter. A task JSON is
recognised by its ``planning-task-info`` key.

If neither is supplied, the request fails with an explicit message rather than
attempting a translation from PDDL.

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
