Muddy Children
==============

The muddy children puzzle is the standard first example of reasoning about
knowledge. Several children have muddy foreheads; each sees the others but not
themselves. A public announcement that at least one is muddy, followed by
repeated rounds in which every child says whether it knows its own state,
eventually lets the muddy ones work it out. Nothing about the world changes in
any round. What changes is what each child knows about what the others know.

The domain is worth running first because it is entirely epistemic: no action
has an ontic effect, so a classical planner has nothing to work with, and every
step of the solution is a change in the model rather than in the facts.

The task
--------

``plansys2_epistemic_planner/test/tasks/muddy-children-2.json`` is the
two-child instance and the smallest solvable task in the repository. Its
``planning-task-info`` records two agents and two atoms, and the requirements
it declares include ``:common-knowledge``, ``:knowing-whether``,
``:modal-goals`` and ``:modal-preconditions``.

``muddy-children-3.json`` is the asymmetric three-child variant, used in the
tests for the determinism check.

Planning without ROS
--------------------

The parser, the search strategies and the policy plan are covered by tests that
need no ROS graph, and running them is the quickest confirmation that the
planner works on this task:

.. code-block:: bash

   colcon test --packages-select plansys2_epistemic_planner
   colcon test-result --verbose

.. todo::

   A standalone command-line entry point for solving a task file outside ROS is
   not installed by ``plansys2_epistemic_planner``: its CMakeLists builds one
   shared library and no executable. Document the invocation here if such a
   tool is added, or remove this section.

The action mapping
------------------

The grounded actions of this instance are named ``ask_c1`` and ``ask_c2``. The
executor expects PlanSys2 action expressions, so a mapping file states the
correspondence. The one shipped with the planner package, at
``examples/mappings/muddy-children-2.json``, is:

.. code-block:: json

   {
     "ask_c1": {"action": "(ask c1)", "duration": 2.0},
     "ask_c2": {"action": "(ask c2)", "duration": 2.0}
   }

The durations are properties of the robot's implementation of ``ask``, not of
the plan; the epistemic planner is untimed. They are what the executor
schedules and times out against.

Point the solver at the file with the ``action_mapping`` parameter:

.. code-block:: yaml

   planner:
     ros__parameters:
       EPISTEMIC:
         action_mapping: "/abs/path/to/muddy-children-2.json"

Without it the solver falls back to splitting the grounded name at its first
underscore, which happens to give ``(ask c1)`` for this instance. That is not a
reason to omit the file: the convention cannot recover the parameter order a
PDDL domain declares, and the next domain will not be so forgiving.

Running it
----------

Start the system with the epistemic parameters file, and start the epistemic
state on the same task:

.. code-block:: bash

   ros2 launch plansys2_bringup plansys2_bringup_launch_monolithic.py \
     model_file:=<domain.pddl> \
     params_file:=<share>/plansys2_bringup/params/plansys2_epistemic_params.yaml

   ros2 run plansys2_epistemic_executor epistemic_state_node \
     --ros-args -p task_file:=<share>/plansys2_epistemic_planner/test/tasks/muddy-children-2.json

The PDDL domain still has to declare an ``ask`` action, because that is what
the executor looks up to find the behavior tree driving the hardware. The
epistemic solver never reads it.

.. todo::

   A PDDL domain and a matching action performer for this instance are not
   checked into the repository, so the ``model_file`` above has no concrete
   value to give. Add the domain and the performer under a demo package, or
   cite an existing one, and replace this note with the command that runs them.

What to expect
--------------

Every task the grounder produces from the available EPDDL instances is
single-pointed, so the solution to this one is a chain rather than a branching
policy. It exercises the epistemic guard and the DEL update on every step, but
not ``EpistemicSwitch``. The fixture that does branch is
``coin-in-the-box-multipointed.json``; see :doc:`../concepts/epddl` for why it
is derived by hand rather than grounded.
