EPDDL and grounded tasks
========================

PDDL :cite:p:`fox2003pddl21` describes a single world and cannot state event
models, per-agent observability, or goals about knowledge. ePlanSys takes its
problems in EPDDL :cite:p:`epddl2026guideline`, which can, and plans over the
grounded task that plank :cite:p:`plank` produces from it.

Input routes
------------

``EpistemicPlanSolver`` searches over a *grounded* task in the IεPC JSON format
:cite:p:`iepc2026`. ``plansys2_epddl_grounder`` produces it from EPDDL sources.
The solver takes the task from the first available of:

#. the ``problem`` string of ``planner/get_plan``, if it is a grounded task
   (identified by the key ``planning-task-info``);
#. the ``epddl_domain`` and ``epddl_problem`` parameters, grounded on first use
   and whenever either file changes;
#. the ``task_file`` parameter, an absolute path to a grounded task.

No translation from the PDDL problem held by the domain expert is attempted,
as PDDL cannot express the required structure; a request without an epistemic
problem fails with a message saying so.

The PDDL ``model_file`` required by the launch files is still read by the
domain expert, and the executor uses it to find each action's behavior tree.
The PDDL domain describes the robot's actions; the EPDDL domain describes what
the agents know. Both describe the same mission.

Grounded task format
--------------------

A grounded task is a JSON object with six top-level keys
(:numref:`tab-task-keys`).

.. _tab-task-keys:

.. list-table:: Top-level keys of a grounded task.
   :header-rows: 1
   :widths: 25 75

   * - Key
     - Contents
   * - ``planning-task-info``
     - Problem and domain names, the EPDDL libraries and requirements used,
       and counts of agents, atoms, facts, actions and worlds.
   * - ``language``
     - Vocabulary: agents and atoms.
   * - ``facts``
     - Ground facts of the instance.
   * - ``initial-state``
     - Initial Kripke model, including the ``designated`` worlds.
   * - ``actions``
     - Event model of each grounded action.
   * - ``goal``
     - Goal formula, possibly modal.

The parser tests compare the parsed task against the counts in
``planning-task-info``, i.e. against the grounder's accounting.

Requirements
~~~~~~~~~~~~

Requirement flags record which language features an instance uses. The tasks
in this repository use ``:common-knowledge``, ``:disjunctive-list-formulas``,
``:disjunctive-preconditions``, ``:equality``, ``:events-conditions``,
``:finitary-S5-theories``, ``:group-modalities``, ``:knowing-whether``,
``:list-comprehensions``, ``:lists``, ``:modal-goals``,
``:modal-preconditions``, ``:multi-pointed-models``,
``:negative-list-formulas``, ``:negative-preconditions``,
``:partial-observability`` and ``:static-common-knowledge``.

Grounding
---------

``plansys2_epddl_grounder`` runs plank as a subprocess:

.. code-block:: bash

   plank export -d <domain.epddl> -p <problem.epddl> -l <library.epddl>... \
                -o <output directory>

plank is a source dependency in ``dependency_repos.repos``; building the
workspace builds it, and sourcing the workspace puts it on ``PATH``.
``plank_command`` or ``$PLANK`` select a binary outside the workspace. No
package links against plank. Without it the epistemic packages still build and
pass their tests; only the EPDDL front end is unavailable, and it reports so.

A domain lists the action-type libraries it uses in
``:action-type-libraries``, and plank resolves them only against library files
passed to it. The grounder ships the ``intermediate`` library, which every
domain in this repository uses, and passes it when ``epddl_libraries`` is
empty. Domains using another library must name it.

plank prints the offending line on a specification error and terminates on a
signal without setting an exit status. The grounder therefore determines
success by whether the task file was written, and forwards plank's output as
the error message.

The EPDDL sources installed under ``<share>/plansys2_epddl_grounder/examples``
are the three fleet domains and problems (:doc:`../tutorials/index`) and
muddy children. The corridor scenario also ships its PDDL counterpart,
``robot-fleet-domain.pddl``, which the executor needs for action behavior
trees. Further puzzle fixtures are in the ``epddl-workspace`` of the
`Epistemic-Robotics <https://github.com/HanielUlises/epistemic-robotics>`_
repository. The EPDDL grammar is specified by
:cite:t:`epddl2026guideline` and not restated here.

Formula syntax
--------------

Formulas are exchanged between the planner and the epistemic state as text,
since interned formula identifiers are local to a process. The policy carries
formulas in this grammar, and ``epistemic_state/check_formula`` accepts it:

.. code-block:: text

   formula := atom
            | "(true)" | "(false)"
            | "(not" formula ")"
            | "(and" formula+ ")"   | "(or" formula+ ")"
            | "(K" agent formula ")" | "(B" agent formula ")"
            | "(Kw" agent formula ")"
            | "(C (" agent+ ")" formula ")"

``K`` and ``B`` denote the same modality :math:`[i]` and are both accepted
under either frame, so a policy's goal remains parseable if the task's frame
changes. Atoms are written bare (``blocked``); a parenthesised name is parsed
as an operator.

Agent and atom names are resolved against the loaded task. An unknown name is
an error, since it indicates that the policy and the state refer to different
problems.

Checked-in tasks
----------------

``plansys2_epistemic_planner/test/tasks`` contains the grounded tasks used by the
tests; :numref:`tab-tasks` lists the principal ones.

.. _tab-tasks:

.. list-table:: Principal grounded tasks.
   :header-rows: 1
   :widths: 40 60

   * - File
     - Purpose
   * - ``robot-fleet.json``
     - Corridor scenario: two designated worlds; the smallest task whose
       solution branches. Executed end to end by ``plansys2_tests``.
   * - ``robot-fleet-depot.json``
     - Two-route depot: four designated worlds; nested branching.
   * - ``robot-fleet-stale-prior.json``
     - Corridor with a stale prior: the robots are at the junction and the
       model designates only the clear world. Used by ``test_7`` and the
       state node's repair tests.
   * - ``robot-fleet-stale-prior-lone.json``
     - As above, with ``r2`` away from the junction, so that the policy
       contains a report to ``r2``. Used by ``test_7``.
   * - ``coin-in-the-box.json``
     - Sensing domain with a :math:`K` goal.
   * - ``coin-in-the-box-multipointed.json``
     - Multi-pointed variant derived by hand, for the parser tests.
   * - ``muddy-children-2.json``, ``muddy-children-3.json``,
       ``active-muddy-child.json``
     - Puzzle fixtures with up to 32 initial worlds, for the parser and the
       search.

The puzzle instances ground to single-pointed tasks, so each sensing action has
one possible outcome and AO* returns a chain; none exercises contingent
branching. ``coin-in-the-box-multipointed.json`` was derived from
``coin-in-the-box.json`` by two edits: the initial state designates two worlds,
leaving the coin's side open, and the goal is :math:`\mathit{Kw}_A\,
\mathit{tails}`. The original goal :math:`K_A\, \mathit{tails}` is
unachievable in the world where the coin is heads.

The fleet problems declare ``:multi-pointed-models`` and leave the state of each
corridor unspecified, so the grounder designates one world per combination (2
for the corridor, 4 for the depot, 8 for the survey) and every solution
branches.

Action names
------------

A grounded action name is one token in the task's vocabulary
(``inspect_r1``); a PlanSys2 action expression is a name with parameters
(``(inspect_corridor r1)``). The ``action_mapping`` parameter defines the
correspondence (:doc:`../getting_started/first_plan`).
