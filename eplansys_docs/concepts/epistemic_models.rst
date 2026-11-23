Epistemic models
================

The planner's state is a multi-pointed Kripke model
:cite:p:`vanditmarsch2007del,bolander2017gentle`. This chapter describes its
representation, the update that advances it, and the search over it. The
implementation is the Aletheia planning core :cite:p:`aletheia`, linked by
``plansys2_epistemic_planner``.

State representation
--------------------

An epistemic state is a tuple :math:`s = (W, \{R_i\}_{i \in Ag}, V, W^*)`:
a set of worlds, one accessibility relation per agent, a valuation
:math:`V : W \to 2^P`, and a non-empty set of designated worlds
:math:`W^* \subseteq W`. It is stored in five flat arrays
(:numref:`tab-state-arrays`).

.. _tab-state-arrays:

.. list-table:: Storage of an epistemic state.
   :header-rows: 1
   :widths: 25 30 45

   * - Array
     - Shape
     - Meaning
   * - ``valuation``
     - ``num_worlds`` × ``val_words``
     - :math:`V` as a bit matrix
   * - ``set_of``
     - ``num_agents`` × ``num_worlds``
     - :math:`R_i(w)` as an index into the set table
   * - ``set_begin``
     - ``num_sets + 1``
     - offsets into ``members``
   * - ``members``
     - :math:`\sum` set sizes
     - successor sets, as sorted world lists
   * - ``designated``
     - ``rel_words``
     - :math:`W^*`

Accessibility is stored as a table of successor sets. A bit matrix would cost
:math:`|Ag| \cdot |W|^2` bits regardless of the frame. On K45 frames (transitive and
Euclidean, which includes S5 and KD45) two successor sets of the same agent are
equal or disjoint, and the product update and bisimulation contraction both
preserve the frame, so one agent's sets contain at most :math:`|W|` entries in
total. Modal operators are evaluated once per distinct set:
:math:`[i]\varphi` holds at :math:`w` iff :math:`R_i(w) \subseteq
\mathrm{sat}(\varphi)`.

Formulas
--------

The formula language has nine constructors (:numref:`tab-formulas`).

.. _tab-formulas:

.. list-table:: Formula constructors.
   :header-rows: 1
   :widths: 25 75

   * - Kind
     - Notation
   * - ``Top``, ``Bot``
     - :math:`\top`, :math:`\bot`
   * - ``Atom``
     - :math:`p`
   * - ``Not``, ``And``, ``Or``
     - :math:`\neg\varphi`, :math:`\varphi \wedge \psi`,
       :math:`\varphi \vee \psi`
   * - ``Belief``
     - :math:`[i]\varphi`, written :math:`B_i \varphi` or :math:`K_i \varphi`
       according to the frame
   * - ``Common``
     - :math:`C_G \varphi`
   * - ``Kw``
     - :math:`\mathit{Kw}_i \varphi \equiv [i]\varphi \vee [i]\neg\varphi`

Formulas are hash-consed: the constructors intern into a process-wide
registry, so structurally identical formulas are one object with one dense
identifier. Satisfaction sets are memoised by identifier, and one bottom-up
pass computes the extension of each distinct subformula once per model.
Preconditions, observability conditions and goal conjuncts of a task share
many subformulas, which the memo table exploits.

The registry grows monotonically. ``EpistemicPlanSolver`` clears it at the
start of each solve, when no formula of the previous task is referenced.

Product update
--------------

An action is an event model, and applying it in state :math:`s` is the DEL
product update :math:`s \otimes a` :cite:p:`baltag1998logic`. Three variants
are provided:

``product_update``
   The updated state.

``product_update_with_map``
   The updated state and the dense table
   :math:`(\text{world}, \text{event}) \mapsto \text{new world}`.

``product_update_split``
   One state per designated event, sharing the product model and differing
   only in the designated set. These are the distinguishable outcomes of a
   sensing action, on which a policy branches.

Each variant takes the frame (KD45 or S5) and a world-cap policy.

Bisimulation contraction
------------------------

After each product update the state is contracted to the smallest bisimilar
state (restriction to worlds reachable from :math:`W^*`, then ordered
partition refinement), with worlds renumbered in an order determined by the
structure of the model alone. Two
bisimilar states therefore have byte-identical representations.

The closed list stores a 128-bit structural fingerprint of each contracted
state; duplicate detection costs two integer comparisons.

World-cap policies
------------------

A world-cap policy decides whether a product update may proceed, given the
pre-contraction world count :math:`|W| \cdot |E|`. The mapping from task
properties to a policy is defined in one function,
``make_world_cap_policy``.

In the planner revision pinned by ePlanSys the policy is unbounded for every
task. A cap prunes branches silently, which loses plans and makes exhaustion
an unsound proof of unsolvability; an IεPC :cite:p:`iepc2026` muddy-children
instance has an 8-step plan through 1024 raw worlds. Consequently the size of a
model is bounded only by available memory.

Heuristics
----------

The ``heuristic`` parameter accepts the labels in :numref:`tab-heuristics`.

.. _tab-heuristics:

.. list-table:: Heuristics.
   :header-rows: 1
   :widths: 12 88

   * - Label
     - Estimate
   * - ``wc``
     - Number of designated worlds.
   * - ``ug``
     - Number of unsatisfied goal conjuncts.
   * - ``ed``
     - Epistemic distance: for each unsatisfied conjunct :math:`[i]\varphi`,
       the number of worlds accessible from designated worlds where
       :math:`\varphi` fails.
   * - ``ks``
     - Knowledge spread: for each unsatisfied :math:`\mathit{Kw}` conjunct,
       the number of the agent's accessible worlds that do not resolve it.
       Intended for conjunctions of :math:`\mathit{Kw}` goals over several
       agents.
   * - ``rpg``
     - Relaxed announcement closure, maximum over goal conjuncts.
   * - ``radd``
     - Relaxed announcement closure, sum over goal conjuncts. Inadmissible when
       conjuncts share work, but more informative on conjunctive goals.

The first four are goal-counting estimates of the distance between the current
state and the goal. They do not estimate remaining work and are constant on
domains where several actions must be chained before any conjunct changes.

``rpg`` and ``radd`` relax the problem differently from classical planning.
The delete relaxation :cite:p:`hoffmann2001ff` exploits monotone growth of a
fact set. In DEL, knowledge is established by announcements and sensing, which
have no ontic effect and act by eliminating worlds; the monotone quantity is
the shrinking world set. The relaxation applies every eliminating event at
once and ignores interference:

.. math::

   W_0 = W, \qquad
   W_{k+1} = W_k \cap \bigcap
     \{\, \mathrm{sat}_{M|W_k}(\mathrm{pre}(e)) : a \in A,\ e \in E_d(a) \,\}

where an event is skipped if it would empty :math:`W^*`. Each layer costs one
action, so the first layer at which a conjunct holds is a lower bound on the
steps needed to establish it, analogous to fact levels in a relaxed planning
graph.

The closure ignores the preconditions of announcements, the possibility that
an event that prunes well is inconsistent with the actual world, and ontic
effects that restore uncertainty. Conjuncts it does not resolve fall back to a
residual epistemic distance offset past the last layer, so the estimate
retains a gradient.

Search strategies
-----------------

The ``strategy`` parameter accepts:

``gbfs``
   Greedy best-first search.

``ehc``
   Enforced hill climbing :cite:p:`hoffmann2001ff`.

``aostar``
   AO* :cite:p:`nilsson1980principles` over the AND/OR graph induced by
   sensing actions: OR nodes are action choices, AND nodes are the outcomes of
   a sensing action, all of which must be solved. This strategy produces
   branching policies.

``PlannerStats`` records the reason each successor was discarded: dead end,
duplicate, rejected by the world cap, designated set emptied by KD45 seriality
repair, agent left believing a contradiction (``consistent_beliefs``), or
action inapplicable. Only the last is a property of the domain.

Strategy selection
------------------

The strategy and heuristic are chosen by an ordered rule table evaluated
first-match. Each rule is a conjunction of comparisons on named numeric
features of the task; a rule without conditions always matches and must be
last. Disjunction is expressed by two rules with the same outcome, which keeps
the condition language small enough to validate exhaustively at load time.

Features
   Number of worlds, designated worlds, actions, agents and atoms; whether any
   action has more than one designated event, and the maximum number of
   designated events of an action; goal modal depth, whether the goal is
   :math:`\mathit{Kw}`-only, and whether it has an atomic conjunct; frame (S5
   or KD45); presence of partial observability.

Operators
   ``<=``, ``<``, ``>=``, ``>``, ``==``, ``!=``.

The built-in table reproduces the hand-written selectors it replaced, with
thresholds tuned on one fifteen-instance benchmark suite. As data, it can be
retuned without rebuilding, the deciding rule can be reported for each run, and
the policy can be published with the results. Selection runs once before
search.

``policy_file`` replaces the built-in table. The file contains a ``strategy``
section, a ``heuristic`` section, or both, and is validated on load; an
unknown feature, an unknown outcome label, or an unconditional rule followed
by further rules is an error. A policy that fails validation is an error, not
a fallback to the built-in table.
