A first plan
============

This chapter configures the epistemic solver, brings up the system, and
requests a policy for a task shipped with the repository. It assumes a
workspace built and sourced as in :doc:`installation`.

Problem
-------

The example is the corridor scenario of :doc:`../tutorials/fleet_corridor`: two
survey robots and a corridor that may be blocked. Its EPDDL sources are
installed with ``plansys2_epddl_grounder``:

.. code-block:: text

   <share>/plansys2_epddl_grounder/examples/robot-fleet-domain.epddl
   <share>/plansys2_epddl_grounder/examples/robot-fleet-problem.epddl

The solver grounds the sources before searching: it builds the initial Kripke
model from the problem's finitary S5 theory and instantiates each action's
event model, producing an IεPC JSON task (:doc:`../concepts/epddl`). Grounding
runs on first use and whenever a source changes. It can also be run manually,
to inspect a task or check it into a test:

.. code-block:: bash

   ros2 run plansys2_epddl_grounder ground_epddl \
     -d <share>/plansys2_epddl_grounder/examples/robot-fleet-domain.epddl \
     -p <share>/plansys2_epddl_grounder/examples/robot-fleet-problem.epddl \
     -o /tmp/robot-fleet.json

Grounded tasks in ``plansys2_epistemic_planner/test/tasks`` can be used
directly through ``task_file``; ``robot-fleet.json`` is the output of the
command above.

Solver parameters
-----------------

``plansys2_bringup`` ships ``params/plansys2_epistemic_params.yaml``, which
selects the epistemic solver:

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

The default parameters file selects POPF, so this file must be passed
explicitly. All parameters are listed in :numref:`tab-solver-params`.

Bring-up
--------

.. code-block:: bash

   ros2 launch plansys2_bringup plansys2_bringup_launch_monolithic.py \
     model_file:=<domain.pddl> \
     params_file:=<share>/plansys2_bringup/params/plansys2_epistemic_params.yaml \
     epistemic_state:=True

``model_file`` is required by the launch file and read by the domain expert.
The epistemic solver ignores it; the executor uses it to find the behavior
tree for each action.

``epistemic_state:=True`` starts the ``epistemic_state`` node as a fifth
managed node. Executing a policy requires it, since the tree's knowledge guards
and updates are service calls to it (:ref:`arch-state`). It is configured with
the same problem as the solver:

.. code-block:: yaml

   epistemic_state:
     ros__parameters:
       epddl_domain: "/abs/path/to/domain.epddl"
       epddl_problem: "/abs/path/to/problem.epddl"

The node grounds the sources itself, so the state and the planner do not
depend on each other's start-up order.

The node can also run alone, to inspect a model without a planner:

.. code-block:: bash

   ros2 run plansys2_epistemic_executor epistemic_state_node --ros-args \
     -p epddl_domain:=<domain.epddl> \
     -p epddl_problem:=<problem.epddl>

:doc:`command_line` queries and updates such a node from a terminal.

Perception
~~~~~~~~~~

To report sensing outcomes from an occupancy grid, add
``epistemic_perception:=True`` (requires ``epistemic_state:=True``) and declare
regions:

.. code-block:: yaml

   epistemic_perception:
     ros__parameters:
       regions: ["corridor"]
       corridor:
         boxes: [2.0, -0.5, 8.0, 0.5]
         atom: "blocked"
         atom_true_when_clear: false
         sensing_action: "inspect_r1"
         outcome_when_clear: "e-inspect-clear"
         outcome_when_blocked: "e-inspect-blocked"

A region bound to a sensing action reports as that action's outcome; an
unbound region is reported as a public announcement (:ref:`arch-perception`).

Requesting a plan
-----------------

The solver takes the task from the first available of the request's
``problem`` string (if it is a grounded task), the EPDDL parameters, and
``task_file`` (:doc:`../concepts/epddl`). If both EPDDL sources and
``task_file`` are set, the sources are used and a warning is logged. If none is
available, the request fails; no translation from PDDL is attempted.

.. todo::

   The ``ros2 service call`` invocation for ``planner/get_plan`` (service type
   and request layout) belongs to ``plansys2_planner`` and is not yet
   documented here.

Result
------

The solver validates the policy before returning it. ``conditional_plan``
determines how a branching policy is returned:

``policy``
   Branches are kept in the epistemic fields of ``plansys2_msgs/PlanItem``
   (:numref:`tab-planitem`). Execution requires ``EpistemicBTBuilder``. This is
   the setting in the shipped parameters file.

``flatten``
   Only the branch through the lowest-indexed events is returned, with a
   warning. The result is a sequence executable by a stock PlanSys2 executor,
   valid only if execution follows that branch.

``reject``
   A branching solution is refused.

A non-branching solution is returned identically in all three modes.

Action names
------------

The planner's action names are grounded tokens (``inspect_r1``,
``report-blocked_r1``); the executor expects PlanSys2 action expressions
(``(inspect_corridor r1)``), which it looks up in the PDDL domain. The
``action_mapping`` parameter names a JSON file with the correspondence:

.. code-block:: json

   {
     "inspect_r1":        "(inspect_corridor r1)",
     "goto-junction_r1":  {"action": "(goto_junction r1)", "duration": 30.0}
   }

``duration`` is in seconds, positive, default 1.0. The epistemic planner is
untimed; the duration is a property of the action implementation, used by the
executor for scheduling and timeouts.

Without ``action_mapping``, a grounded name is split at its first underscore
and the remainder read as parameters: ``inspect_r1`` becomes ``(inspect r1)``
and ``report-blocked_r1`` becomes ``(report-blocked r1)``. The convention
cannot recover the parameter order of the PDDL domain, so a mapping file is
required for real actions. An action the mapping does not cover fails the
planning request.
