Executing a policy end to end
=============================

The planner could produce a branching policy and the builder could render one
as a behavior tree, but until this work nothing had ever run one over a live
ROS graph. Two end-to-end tests now do, and the package that turns a policy
into a tree has the tests it never had.

:download:`Read the full report (PDF) <../_static/epistemic_end_to_end_test.pdf>`,
five pages, with the reasoning behind each fix and the execution logs.

What the tests run
------------------

Both missions bring up the whole graph in one process (domain expert,
problem expert, planner with the ``plansys2/EpistemicPlanSolver`` plugin,
executor with ``bt_builder_plugin: EpistemicBTBuilder`` and the epistemic node
library, the ``epistemic_state`` node loaded from the same grounded task, and
the fake action performers) and run the same mission twice with the world in
different states.

.. list-table::
   :header-rows: 1
   :widths: 22 48 30

   * - Test
     - Mission
     - What it proves
   * - ``plansys2_tests/test_5``
     - One corridor of unknown state; a four-node policy that drives out,
       looks, and reports.
     - One run executes ``report_clear`` and the other ``report_blocked``, and
       neither runs the branch it did not take.
   * - ``plansys2_tests/test_6``
     - Two corridors and three robots; a twelve-node policy with a switch
       nested inside another switch's continuation.
     - The two runs report the corridors in opposite states and share no leaf,
       so the second half of the mission is correct inside either branch of the
       first.
   * - ``plansys2_epistemic_bt_builder/test``
     - Twelve unit tests of the plugin's contract with ``plansys2_executor``.
     - Switch rendering and outcome order, flat rendering of unbranched plans,
       refusal of a malformed policy, agreement with
       ``BTBuilder::to_action_id``, the dotgraph, and pluginlib lookup by name.

Where the observation comes from
--------------------------------

The epistemic state can name a sensing outcome by itself only when its model
already designates one world. Both fleet fixtures designate several, which
is what makes them worth testing with, and what means the observation has to
come from whoever did the sensing.

``ApplyEpistemicUpdate`` takes it on an ``observed`` port that the packaged
template leaves unbound, because what a robot saw is domain-specific and
PlanSys2 carries nothing back from a performer. The tests bind it as a
deployment would: an action template with the port wired to ``ObservedOutcome``,
a node that reads ``action=outcome`` entries from a latched topic and reports
the one matching its own action.

What the model then does is the point of the whole stack. From the depot run
with north blocked and south clear::

    [epistemic_state] applied inspect-north_r1 -> e-inspect-north-blocked: 4 worlds, 2 designated
    [epistemic_state] applied inspect-south_r2 -> e-inspect-south-clear:   4 worlds, 1 designated
    [epistemic_bt] [goal] (and (Kw r1 blocked-north) ... (Kw r3 blocked-south)) holds

Four candidate worlds at the start, two after the first sensing, one after the
second, and the goal checked against the state that actually resulted rather
than the one planning assumed.

Defects the end-to-end path exposed
-----------------------------------

None of these is visible to a unit test, and each is fatal to a real execution.
All four are fixed; the report has the details.

.. list-table::
   :header-rows: 1
   :widths: 6 64 30

   * - \#
     - Defect
     - Where
   * - 1
     - A builder loaded by name was never initialized: ``initialize()`` was
       called only for ``SimpleBTBuilder`` and ``STNBTBuilder``, so any plugin
       builder ran with its action template and time precision discarded.
     - ``ExecutorNode.cpp``
   * - 2
     - With (1) fixed, the builder could be handed PlanSys2's default action
       template, which has no ``CONTINUATIONS`` placeholder: the rendered tree
       holds only the root action, runs, succeeds, and leaves the mission
       undone. Such a template is now refused, with a warning.
     - ``epistemic_bt_builder.cpp``
   * - 3
     - An empty policy, a goal that already holds, rendered a childless
       ``Sequence``, which BehaviorTree.CPP rejects, so "nothing to do" crashed
       tree creation. It now renders ``AlwaysSuccess``.
     - ``policy_bt.cpp``
   * - 4
     - ``nlohmann_json`` sits in the exported link interfaces of the planner
       and the executor but was absent from ``ament_export_dependencies``, so
       the first consumer that does not find it independently failed to
       configure.
     - ``CMakeLists.txt`` (×2)

Results
-------

Verified in the ``ros:rolling`` container, the environment the ``rolling``
workflow builds in.

.. list-table::
   :header-rows: 1
   :widths: 40 12 12 18

   * - Suite
     - Tests
     - Result
     - Time
   * - ``epistemic_bt_builder_test``
     - 12
     - pass
     - 0.16 s
   * - ``test_5`` (corridor)
     - 2
     - pass
     - 32.3 s
   * - ``test_6`` (depot)
     - 2
     - pass
     - 56.6 s
   * - ``plansys2_epistemic_executor`` (existing)
     - 35
     - pass
     - none
   * - ``plansys2_tests`` (whole package, lint included)
     - 165
     - pass
     - 3 min 18 s
   * - ``plansys2_executor`` (whole package, lint included)
     - 255
     - pass
     - 2 min 57 s

``plansys2_epistemic_bt_builder`` is also removed from the ``packages-ignore``
list of the coverage step in ``rolling.yaml``: that list is for packages
producing no ``.gcda`` at all, and with a test suite the builder now produces
one.

Known gaps
----------

.. todo::

   The negative case is not covered: an outcome the policy does not plan for,
   where ``EpistemicSwitch`` must fail rather than guess. It needs a performer
   that reports something the model cannot account for.

* ``ObservedOutcome`` is a test's stand-in for a sensor. A deployment still
  binds ``observed`` to its own performers, the one piece of wiring a domain
  supplies itself.
* The tests build against ROS 2 rolling only. The Humble job stops before
  ``plansys2_executor``, as it did before this work.
* The mission fixtures ``robot-fleet.json`` and ``robot-fleet-depot.json``, with
  their action mappings, come from the ``feature/epddl-front-end`` branch.
