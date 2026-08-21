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

The problem
-----------

The two-child instance is the smallest solvable problem in the repository, and
its EPDDL sources are packaged with the grounder:

.. code-block:: text

   <share>/plansys2_epddl_grounder/examples/muddy-children-domain.epddl
   <share>/plansys2_epddl_grounder/examples/muddy-children-problem.epddl

The domain is short enough to read in full. One predicate, two events for the
two answers a child can give, and one action pairing them:

.. code-block:: text

   (:predicates (muddy ?i - agent))

   (:event e-ask-pos
     :parameters (?i - agent)
     :precondition (and (muddy ?i) (<Kw. ?i> (muddy ?i))))

   (:event e-ask-neg
     :parameters (?i - agent)
     :precondition (and (not (muddy ?i)) (<Kw. ?i> (muddy ?i))))

   (:action ask
     :parameters (?i - agent)
     :action-type (public-sensing (e-ask-pos ?i) (e-ask-neg ?i))
     :observability-conditions (default Fully))

``<Kw. ?i>`` is "it is not the case that ``?i`` knows whether", so a child can
answer only about a forehead it cannot yet settle — which is what makes the
puzzle advance. ``public-sensing`` and ``Fully`` come from the ``intermediate``
action-type library, which the grounder supplies.

The problem file names the agents, states the initial situation as a finitary
S5 theory, and sets a modal goal:

.. code-block:: text

   (:agents c1 c2)

   (:init
     (:and
       (muddy c1) (muddy c2)
       (:forall (?i ?j - agent | (/= ?i ?j))
         ([C. All] ([Kw. ?i] (muddy ?j))))))

   (:goal
     (and ([Kw. c1] (muddy c1)) ([Kw. c2] (muddy c2))))

Both children are muddy, it is common knowledge that each knows whether the
*other* is muddy, and the goal is that each comes to know whether it is muddy
itself. Grounding turns that theory into the four worlds — one per assignment
of muddiness — and the two accessibility relations that make each child unable
to tell its own state apart.

Grounding it by hand shows exactly that:

.. code-block:: bash

   ros2 run plansys2_epddl_grounder ground_epddl \
     -d <share>/plansys2_epddl_grounder/examples/muddy-children-domain.epddl \
     -p <share>/plansys2_epddl_grounder/examples/muddy-children-problem.epddl \
     -o /tmp/muddy-children-2.json

The result is byte-for-byte the checked-in
``plansys2_epistemic_planner/test/tasks/muddy-children-2.json``: two agents,
two atoms, two grounded actions and four initial worlds, declaring
``:common-knowledge``, ``:knowing-whether``, ``:modal-goals`` and
``:modal-preconditions`` among its requirements.

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

Copy the epistemic parameters file, and fill the same two paths into both the
solver's parameters and the epistemic state's:

.. code-block:: yaml

   planner:
     ros__parameters:
       EPISTEMIC:
         epddl_domain: "<share>/plansys2_epddl_grounder/examples/muddy-children-domain.epddl"
         epddl_problem: "<share>/plansys2_epddl_grounder/examples/muddy-children-problem.epddl"
         action_mapping: "<share>/plansys2_epistemic_planner/examples/mappings/muddy-children-2.json"

   epistemic_state:
     ros__parameters:
       epddl_domain: "<share>/plansys2_epddl_grounder/examples/muddy-children-domain.epddl"
       epddl_problem: "<share>/plansys2_epddl_grounder/examples/muddy-children-problem.epddl"

Then one command starts the whole system, epistemic state included:

.. code-block:: bash

   ros2 launch plansys2_bringup plansys2_bringup_launch_monolithic.py \
     model_file:=<domain.pddl> \
     params_file:=<your copy of plansys2_epistemic_params.yaml> \
     epistemic_state:=True

The model it comes up holding is the four-world one the ``:init`` theory
describes, and it can be questioned directly:

.. code-block:: bash

   ros2 service call /epistemic_state/check_formula \
     plansys2_epistemic_msgs/srv/CheckFormula "{formula: '(Kw c1 muddy_c1)'}"

Before any asking, that answers ``holds: false`` — which is the puzzle: both
children are muddy, and neither yet knows it of itself.

The same conversation is easier through the CLI, which talks to the epistemic
state the way the terminal talks to the problem expert:

.. code-block:: bash

   ros2 plansys2 epistemic show
   ros2 plansys2 epistemic check "(K c1 muddy_c1)"      # does not hold
   ros2 plansys2 epistemic announce "muddy_c1"          # 4 worlds -> 2
   ros2 plansys2 epistemic check "(K c1 muddy_c1)"      # holds

That is the puzzle's own mechanism in three commands. Announcing does not make
``c1`` muddy — it already was, in the designated world — it rules out the two
worlds where it was not, and ``c1`` could not previously tell those apart from
the others. The father's announcement in the story does exactly this.

The goal can be changed the same way, and the next plan is built for it:

.. code-block:: bash

   ros2 plansys2 epistemic goal                          # the task's own
   ros2 plansys2 epistemic goal "(Kw c1 muddy_c1)"       # only c1 need find out

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
