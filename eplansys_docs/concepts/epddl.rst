EPDDL and Grounded Tasks
========================

Classical PlanSys2 takes a PDDL domain and problem. An epistemic domain needs
more than PDDL can state: event models, per-agent observability, and goals that
are formulas about knowledge rather than about facts. EPDDL is the language
that states them, and the planner in this repository consumes its grounded
output.

What the solver reads
---------------------

``EpistemicPlanSolver`` searches over a *grounded* epistemic planning task in
the IePC JSON format. EPDDL is what a user writes; the grounded task is what
the search runs on, and ``plansys2_epddl_grounder`` is the step between them.

There is deliberately no translation from the PDDL problem the domain expert
holds: PDDL cannot express event models or per-agent observability, so no such
translation exists. A request that supplies no epistemic problem at all fails
with that explanation rather than with a parse error.

The task arrives by the first of three routes that is available:

* the ``problem`` string of ``planner/get_plan``, when that string is itself a
  grounded task JSON, recognised by its ``planning-task-info`` key;
* the ``epddl_domain`` and ``epddl_problem`` parameters, ground on first use
  and again whenever the files change;
* the ``task_file`` parameter, an absolute path to a grounded task on disk.

The ``model_file`` the launch file requires still feeds the domain expert, and
is what the executor consults to find the behavior tree for an action. The
epistemic solver ignores it: the PDDL states what the robot can do, the EPDDL
states what the agents know.

Structure of a grounded task
----------------------------

A grounded task is a JSON object with six top-level keys:

.. list-table::
   :header-rows: 1
   :widths: 25 75

   * - Key
     - Contents
   * - ``planning-task-info``
     - Problem and domain names, the EPDDL libraries and requirements the
       instance uses, and counts of agents, atoms, facts, actions and worlds.
   * - ``language``
     - The vocabulary: the agents and the atoms formulas are written over.
   * - ``facts``
     - The instance's ground facts.
   * - ``initial-state``
     - The initial Kripke model, including its ``designated`` worlds.
   * - ``actions``
     - The event model of each grounded action.
   * - ``goal``
     - The goal formula, which may be modal.

The count fields in ``planning-task-info`` are what the parser tests assert
against, so a task file is checked against the grounder's own accounting rather
than against the parser's reading of it alone.

Requirements
------------

The requirement flags a grounded task declares record which parts of the
language the instance uses. Those appearing in the tasks checked into this
repository include ``:common-knowledge``, ``:disjunctive-list-formulas``,
``:disjunctive-preconditions``, ``:equality``, ``:events-conditions``,
``:finitary-S5-theories``, ``:group-modalities``, ``:knowing-whether``,
``:list-comprehensions``, ``:lists``, ``:modal-goals``,
``:modal-preconditions``, ``:multi-pointed-models``,
``:negative-list-formulas``, ``:negative-preconditions``,
``:partial-observability`` and ``:static-common-knowledge``.

Grounding
---------

Grounding is done by `plank <https://github.com/HanielUlises/plank>`_, the
EPDDL toolkit, which ``plansys2_epddl_grounder`` runs as a subprocess:

.. code-block:: bash

   plank export -d <domain.epddl> -p <problem.epddl> -l <library.epddl>... \
                -o <output directory>

plank is listed in ``dependency_repos.repos``, so building the workspace builds
it and sourcing the workspace puts it on PATH; the ``plank_command`` parameter
and the ``PLANK`` environment variable name it explicitly for an installation
outside the workspace. Nothing else in the tree links against it: the
epistemic packages build and their tests pass without it, and only the EPDDL front end is
unavailable, which it reports rather than failing silently.

A domain declares the action-type libraries it uses in its
``:action-type-libraries`` clause, and plank resolves those names only against
library files it is handed. ``plansys2_epddl_grounder`` therefore ships a copy
of the ``intermediate`` library, which every domain in this repository declares,
and supplies it when ``epddl_libraries`` is empty. A domain using a different
library must name it.

Two properties of plank's error reporting are worth knowing, because they shape
what a grounding failure looks like in a ROS log: it prints the offending line
of the specification, and it terminates on a signal rather than exiting with a
status. The grounder therefore judges success by whether the task file was
written, and passes plank's own output through as the error.

The EPDDL sources are packaged under
``<share>/plansys2_epddl_grounder/examples``: the three fleet domains and
their problems, described in :doc:`../tutorials/index`, and muddy children,
which is the puzzle the fleet scenarios replaced as the worked example. The
corridor scenario also ships its PDDL side, ``robot-fleet-domain.pddl``, since
the executor needs a behavior tree per action and the EPDDL says nothing about
hardware: the two files describe one mission from two sides. The remaining
puzzle fixtures live in the ``epddl-workspace`` of the
`Epistemic-Robotics <https://github.com/HanielUlises/epistemic-robotics>`_
repository. The EPDDL grammar itself is
specified in the `EPDDL Official Guideline <https://arxiv.org/abs/2601.20969>`_
and is not restated here.

Formula syntax
--------------

Formulas travel between the planner and the epistemic state as text: they have
to cross a service boundary, and the interned identifiers they were built from
are process-local. The grammar is the one the policy carries and the one
``epistemic_state/check_formula`` accepts:

.. code-block:: text

   formula := atom
            | "(true)" | "(false)"
            | "(not" formula ")"
            | "(and" formula+ ")"   | "(or" formula+ ")"
            | "(K" agent formula ")" | "(B" agent formula ")"
            | "(Kw" agent formula ")"
            | "(C (" agent+ ")" formula ")"

``K`` and ``B`` are the same modality under different frames, an agent's box,
and both are accepted whatever the task's frame is. Rejecting ``(K ...)`` on a
doxastic task would only mean that the goal a policy carries stops parsing the
moment the frame changes.

Agent and atom names are resolved against the loaded task, so a formula naming
a symbol the task does not have is an error rather than a fresh symbol: it
means the policy and the state disagree about which problem is being solved.

Checked-in tasks
----------------

.. list-table::
   :header-rows: 1
   :widths: 40 60

   * - File
     - Why it exists
   * - ``robot-fleet.json``
     - The corridor scenario: two designated worlds, and the smallest task
       whose solution branches. Executed end to end by ``plansys2_tests``.
   * - ``robot-fleet-depot.json``
     - The two-route depot: four designated worlds, and a policy with a branch
       inside a branch.
   * - ``coin-in-the-box.json``
     - Sensing domain with a knowledge goal rather than a
       :math:`\mathit{Kw}` goal.
   * - ``coin-in-the-box-multipointed.json``
     - The hand-derived multi-pointed coin, kept for the parser tests.
   * - ``muddy-children-2.json``, ``muddy-children-3.json``,
       ``active-muddy-child.json``
     - Puzzle fixtures that predate the fleet scenarios; they pin the parser
       and the search against tasks with up to thirty-two initial worlds.

The puzzle instances all ground to single-pointed tasks, one designated
world, so sensing has exactly one possible outcome in them and AO* returns a
chain. Nothing among them exercises contingent branching, which is precisely
the path the solver flattens away when ``conditional_plan`` is not ``policy``.
``coin-in-the-box-multipointed.json`` was derived by hand to cover that gap,
from ``coin-in-the-box.json`` with two edits: the initial state designates two
worlds rather than one, so it is genuinely open whether the coin lies tails;
and the goal is :math:`\mathit{Kw}_A(\mathit{tails})` rather than
:math:`K_A(\mathit{tails})`, since knowing that the coin is tails is
unachievable in the world where it is not.

The fleet problems close that gap properly. They declare
``:multi-pointed-models`` and state an initial theory that says nothing about
whether a corridor is blocked, so the grounder produces a task designating one
world per combination (two for the corridor, four for the depot, eight for
the survey) and every solution to them branches.

Action names
------------

A grounded epistemic action name is a single token in the task's own
vocabulary. PlanSys2 action expressions are a different vocabulary, not a
different spelling of the same one, and the ``action_mapping`` parameter states
the correspondence between them. See :doc:`../getting_started/first_plan` for
the file format and the fallback convention.
