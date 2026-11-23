End-to-end policy execution
===========================

This report describes the integration tests that execute branching epistemic
policies over a live ROS graph, the unit tests of the behavior tree builder
plugin, and the defects the end-to-end path exposed.

:download:`Full report (PDF, five pages) <../_static/epistemic_end_to_end_test.pdf>`,
with the analysis of each defect and the execution logs.

Test setup
----------

Each mission brings up the complete graph in one process: domain expert,
problem expert, planner with ``plansys2/EpistemicPlanSolver``, executor with
``bt_builder_plugin: EpistemicBTBuilder`` and the epistemic node library, the
``epistemic_state`` node loaded with the same grounded task, and stand-in
action performers. Each mission runs once per world state of interest
(:numref:`tab-e2e-tests`).

.. _tab-e2e-tests:

.. list-table:: Test suites.
   :header-rows: 1
   :widths: 22 48 30

   * - Test
     - Mission
     - Property verified
   * - ``plansys2_tests/test_5``
     - One corridor of unknown state; four-node policy (drive, inspect,
       report).
     - Each run executes exactly the report for the observed state
       (``report_clear`` or ``report_blocked``).
   * - ``plansys2_tests/test_6``
     - Two corridors, three robots; twelve-node policy with a switch nested in
       another switch's continuation.
     - Runs with the corridors in opposite states reach disjoint leaves; the
       second half is correct under either branch of the first.
   * - ``plansys2_tests/test_7``
     - The corridor mission against a world that deviates from the model.
     - No action planned for a refuted belief executes; under
       ``MissionSupervisor`` the mission completes from the repaired belief.
   * - ``plansys2_epistemic_bt_builder/test``
     - Twelve unit tests of the plugin's contract with ``plansys2_executor``.
     - Switch rendering and outcome order, flat rendering of unbranched plans,
       rejection of malformed policies, agreement with
       ``BTBuilder::to_action_id``, dot graph, pluginlib lookup by name.

Observation routes
------------------

The epistemic state determines a sensing outcome by itself only if its model
designates a single world. The fleet tasks designate several, so the outcome
must be supplied by the sensing agent. The two routes of
:ref:`arch-observation` are each covered by one test.

``test_5``
   Performer route. The stand-in ``inspect_corridor`` performer reports the
   corridor state in ``ActionExecution.outcome``; the packaged action template
   binds ``ExecuteAction``'s outcome port to ``ApplyEpistemicUpdate``'s
   ``observed``. Executor defaults are used.

``test_6``
   External route, for observations not originating from the acting
   performer. The test supplies an action template that binds ``observed`` to
   ``ObservedOutcome``, a node reading ``action=outcome`` entries from a
   latched topic.

Log of the depot run with north blocked and south clear::

    [epistemic_state] applied inspect-north_r1 -> e-inspect-north-blocked: 4 worlds, 2 designated
    [epistemic_state] applied inspect-south_r2 -> e-inspect-south-clear:   4 worlds, 1 designated
    [epistemic_bt] [goal] (and (Kw r1 blocked-north) ... (Kw r3 blocked-south)) holds

The designated set shrinks from four worlds to two and then one, and the goal
is evaluated in the resulting state.

Defects found
-------------

The defects of :numref:`tab-e2e-defects` are invisible to unit tests and each
prevents a correct execution. All are fixed.

.. _tab-e2e-defects:

.. list-table:: Defects exposed by the end-to-end path.
   :header-rows: 1
   :widths: 6 64 30

   * - \#
     - Defect
     - Location
   * - 1
     - A builder loaded by name was not initialised: ``initialize()`` was
       called only for ``SimpleBTBuilder`` and ``STNBTBuilder``, so plugin
       builders ran without their action template and time precision.
     - ``ExecutorNode.cpp``
   * - 2
     - After (1), the builder could receive PlanSys2's default action
       template, which lacks the ``CONTINUATIONS`` placeholder; the tree then
       contained only the root action and succeeded with the mission
       incomplete. Such a template is now rejected with a warning.
     - ``epistemic_bt_builder.cpp``
   * - 3
     - An empty policy (goal already satisfied) rendered a childless
       ``Sequence``, which BehaviorTree.CPP rejects. It now renders
       ``AlwaysSuccess``.
     - ``policy_bt.cpp``
   * - 4
     - ``nlohmann_json`` was in the exported link interfaces of the planner and
       the executor but missing from ``ament_export_dependencies``, so
       downstream packages failed to configure.
     - ``CMakeLists.txt`` (×2)

Results
-------

.. list-table:: Results in the ``ros:rolling`` container (the environment of
   the ``rolling`` workflow).
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
     - n/a
   * - ``plansys2_tests`` (package, including linters)
     - 165
     - pass
     - 3 min 18 s
   * - ``plansys2_executor`` (package, including linters)
     - 255
     - pass
     - 2 min 57 s

``test_5``, ``test_6`` and ``test_7`` also pass on ROS 2 Humble, where the
complete stack builds. ``plansys2_epistemic_bt_builder`` was removed from the
``packages-ignore`` list of the coverage step in ``rolling.yaml``, which lists
packages producing no ``.gcda`` files.

Limitations
-----------

* ``test_7`` covers outcomes outside the model: an outcome no event describes
  stops the mission at the update, before either branch; an outcome the
  planner held impossible leads to model repair, after which the knowledge
  guard rejects the action planned for the old belief. Neither case reaches
  ``EpistemicSwitch``, whose rejection of unplanned outcomes is covered by
  unit tests only: with two-outcome sensing, a policy either branches on both
  outcomes or commits to one and has no switch.
* ``ObservedOutcome`` is a test stand-in for a sensor. A deployment binds
  ``observed`` to its own performers.
