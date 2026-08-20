EPDDL and Grounded Tasks
========================

Classical PlanSys2 takes a PDDL domain and problem. An epistemic domain needs
more than PDDL can state: event models, per-agent observability, and goals that
are formulas about knowledge rather than about facts. EPDDL is the language
that states them, and the planner in this repository consumes its grounded
output.

What the solver reads
---------------------

``EpistemicPlanSolver`` reads a grounded epistemic planning task in the IePC
JSON format. It does not parse EPDDL, and it does not attempt to translate the
PDDL problem held by the domain expert: PDDL cannot express event models or
per-agent observability, so no such translation exists. A request that supplies
neither a task JSON nor a ``task_file`` fails with that explanation rather than
with a parse error.

The task arrives by whichever of two routes comes first:

* the ``problem`` string of ``planner/get_plan``, when that string is itself a
  grounded task JSON, recognised by its ``planning-task-info`` key, or
* the ``task_file`` parameter, an absolute path to the same thing on disk.

The ``model_file`` the launch file requires still feeds the domain expert, and
is what the executor consults to find the behavior tree for an action. The
epistemic solver ignores it.

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

The grounded tasks in this repository were produced from EPDDL sources by an
external grounder and are checked in verbatim. The EPDDL sources live in the
``epddl-workspace`` of the `Epistemic-Robotics
<https://github.com/HanielUlises/epistemic-robotics>`_ repository, and the
grounder is invoked as ``plank ground``.

.. todo::

   The EPDDL grammar, and the installation and full command line of the
   ``plank`` grounder, are not part of this repository and are not documented
   here. Add a reference to the grounder's own documentation, or a summary of
   the subset of EPDDL these tasks use, once that source is fixed.

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
   * - ``muddy-children-2.json``
     - Smallest solvable task; the default for the search tests.
   * - ``muddy-children-3.json``
     - Asymmetric variant, used for the determinism check.
   * - ``coin-in-the-box.json``
     - Sensing domain with a knowledge goal rather than a
       :math:`\mathit{Kw}` goal.
   * - ``active-muddy-child.json``
     - Thirty-two initial worlds; the canonical partial-observability case.
   * - ``coin-in-the-box-multipointed.json``
     - The only fixture whose solution is a branching policy.

Every task the grounder produces from the available EPDDL instances is
single-pointed, with one designated world, so sensing has exactly one possible
outcome and AO* returns a chain. Nothing in the workspace exercises contingent
branching, which is precisely the path the solver flattens away when
``conditional_plan`` is not ``policy``. ``coin-in-the-box-multipointed.json``
is therefore derived by hand from ``coin-in-the-box.json`` with two edits: the
initial state designates two worlds rather than one, so it is genuinely open
whether the coin lies tails; and the goal is :math:`\mathit{Kw}_A(\mathit{tails})`
rather than :math:`K_A(\mathit{tails})`, since knowing that the coin is tails is
unachievable in the world where it is not. Its solution branches. If the EPDDL
instance is ever written and grounded, the file should be replaced with the
grounder's output.

Action names
------------

A grounded epistemic action name is a single token in the task's own
vocabulary. PlanSys2 action expressions are a different vocabulary, not a
different spelling of the same one, and the ``action_mapping`` parameter states
the correspondence between them. See :doc:`../getting_started/first_plan` for
the file format and the fallback convention.
