Epistemic Models
================

The planner's state is a multi-pointed Kripke model. This page describes how
that model is represented, how it is advanced, and what the search does with
it.

State representation
--------------------

An epistemic state is a tuple :math:`(W, \{R_i\}_{i \in Ag}, V, W^*)`: a set of
possible worlds, one accessibility relation per agent, a valuation assigning
atoms to worlds, and a set of designated worlds, the ones that may be the
actual one.

The implementation stores exactly three flat word arrays:

.. list-table::
   :header-rows: 1
   :widths: 25 30 45

   * - Array
     - Shape
     - Meaning
   * - ``valuation``
     - ``num_worlds`` × ``val_words``
     - :math:`V : W \to 2^P` as a bit matrix
   * - ``relation``
     - ``num_agents`` × ``num_worlds`` × ``rel_words``
     - each :math:`R_i \subseteq W \times W`, row-major by source world
   * - ``designated``
     - ``rel_words``
     - :math:`W^* \subseteq W`

Two consequences follow. Modal operators become word-parallel:
:math:`[i]\varphi` holds at :math:`w` exactly when
:math:`R_i(w) \wedge \neg\mathrm{sat}(\varphi)` is empty, which costs
:math:`\lceil |W|/64 \rceil` tests regardless of how many successors :math:`w`
has. And the whole model is three contiguous allocations, so copying a state is
three block copies and hashing it is a linear scan.

Formulas
--------

The formula language has nine constructors:

.. list-table::
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
       depending on the frame
   * - ``Common``
     - :math:`C_G \varphi`
   * - ``Kw``
     - :math:`\mathit{Kw}_i \varphi \equiv [i]\varphi \vee [i]\neg\varphi`

Formulas are hash-consed: the factories intern into a process-wide registry, so
two structurally identical formulas are the same object and carry the same
dense identifier. Satisfaction-set evaluation memoises by that identifier, and
since a task's preconditions, postcondition guards, observability conditions
and goal conjuncts share many subformulas, interning turns that sharing into
cache hits. One bottom-up pass computes each distinct subformula's extension
once per model.

The registry only grows, so a long-lived planner node clears it between tasks.
``EpistemicPlanSolver`` does so at the start of each solve, once nothing from
the previous one is alive.

Product update
--------------

An action is an event model, and applying it is the DEL product update
:math:`s \otimes a`. Three variants exist:

``product_update``
   The updated state.

``product_update_with_map``
   The updated state together with the dense
   :math:`(\text{world}, \text{event}) \to \text{new world}` table used to
   build it.

``product_update_split``
   The sensing case: one state per designated event, all sharing the same
   product model and differing only in which worlds are designated. This is
   what makes an action's outcomes distinguishable, and therefore what a policy
   branches on.

Each takes a flag selecting KD45 rather than S5 semantics, and a world cap
policy.

Bisimulation contraction
------------------------

After every product update the state is contracted to the smallest bisimilar
state, with worlds renumbered in an order that depends only on the model's
structure and never on the numbering the caller supplied. Two bisimilar states
therefore come back byte-identical.

That canonical labelling is what makes the closed list cheap. A contracted
state costs kilobytes; the search stores a 128-bit structural fingerprint
instead, and comparison is two integer tests rather than a graph walk.

World cap policies
------------------

A world cap policy decides at run time whether a proposed product update may
proceed, given the raw pre-contraction world count :math:`|W| \cdot |E|`. It is
constructed once per search from the task and consulted on every update.

``BoundedWorldCap``
   Rejects an update whose raw product exceeds the cap. Used where nil-event
   accumulation causes exponential growth.

``UnboundedWorldCap``
   Always allows. Used for partial-observability domains, where the world count
   grows roughly linearly per step and contraction keeps it bounded in
   practice.

The mapping from task properties to a policy lives in one function,
``make_world_cap_policy``, rather than at each call site.

Heuristics
----------

Six heuristics are available, named by the labels the ``heuristic`` parameter
accepts:

.. list-table::
   :header-rows: 1
   :widths: 12 88

   * - Label
     - Estimate
   * - ``wc``
     - Number of designated worlds.
   * - ``ug``
     - Number of goal conjuncts not yet satisfied.
   * - ``ed``
     - Epistemic distance. For each unsatisfied belief conjunct
       :math:`[i]\varphi`, the number of accessible worlds, from designated
       worlds, where :math:`\varphi` fails. Gives a gradient where ``ug`` sees
       only zero or one per conjunct.
   * - ``ks``
     - Knowledge spread. For each unsatisfied :math:`\mathit{Kw}` conjunct, how
       many of the agent's accessible worlds still fail to resolve it. Intended
       for goals that are conjunctions of :math:`\mathit{Kw}` formulas across
       several agents.
   * - ``rpg``
     - Relaxed announcement closure, aggregated by maximum over goal conjuncts.
   * - ``radd``
     - The same closure, aggregated by sum. Over-estimates when conjuncts share
       work, but guides conjunctive goals considerably better.

The first four are goal counting: they measure how wrong the current state is,
not how much work remains, and are flat on any domain where several actions
must be chained before the first conjunct becomes true.

The relaxation behind ``rpg`` and ``radd`` is specific to epistemic planning.
In classical planning one relaxes by ignoring delete effects, because progress
is monotone growth of a fact set. In DEL the actions that establish knowledge
are announcements and sensing, which carry no ontic effect at all: they make
progress by eliminating worlds an agent considers possible. The monotone
quantity is therefore the world set, shrinking rather than growing, and the
relaxation applies every eliminating action at once, ignoring the interference
between them:

.. math::

   W_0 = W, \qquad
   W_{k+1} = W_k \cap \bigcap
     \{\, \mathrm{sat}_{M|W_k}(\mathrm{pre}(e)) : a \in A,\ e \in E_d(a) \,\}

where an event is skipped if including it would empty :math:`W^*`. Each layer
costs one action in the relaxed plan, so the layer at which a goal conjunct
first becomes true is a lower bound on the number of steps needed to establish
it: the epistemic analogue of a relaxed planning graph's fact levels.

The closure ignores that announcements have preconditions of their own, that
events pruning well in the relaxation may be inconsistent with the actual
world, and that ontic effects can restore uncertainty. Conjuncts it cannot
resolve fall back to a residual epistemic distance offset past the last layer,
so the estimate degrades to a gradient rather than to a constant.

Search strategies
-----------------

Three strategies are available, named by the labels the ``strategy`` parameter
accepts:

``gbfs``
   Greedy best-first search.

``ehc``
   Enforced hill climbing.

``aostar``
   AO* over the AND/OR graph induced by sensing actions. This is the strategy
   that produces a branching policy rather than a sequence: the OR nodes are
   action choices and the AND nodes are the outcomes of a sensing action, all
   of which must be handled.

The search records why branches were discarded, not merely how many.
``PlannerStats`` separates dead ends, duplicates, updates rejected by the world
cap, updates whose designated set was emptied by KD45 seriality repair, and
genuinely inapplicable actions. Only the last is a property of the domain, and
collapsing them makes a failed run unreadable.

Selection policy
----------------

Which strategy and heuristic to use is decided by a rule table rather than by
control flow. Rules are evaluated first-match-wins over an ordered list; each
rule is a conjunction of comparisons against named numeric features of the
task, and a rule with no conditions always matches and so acts as a terminal
default. Disjunction is expressed by listing two rules with the same outcome,
which keeps the condition language small enough to validate exhaustively at
load time.

The features a rule may test are the number of worlds, designated worlds,
actions, agents and atoms; whether any action has more than one designated
event, and the maximum number of designated events over actions; the goal's
modal depth, whether it is :math:`\mathit{Kw}`-only, and whether it has an atom
conjunct; and whether the frame is S5 or KD45 and whether the domain has
partial observability.

The comparison operators are ``<=``, ``<``, ``>=``, ``>``, ``==`` and ``!=``.

The built-in table reproduces the hand-written selectors it replaced, and its
thresholds are tuned to one fifteen-instance benchmark suite. Encoding it as
data makes three things possible that control flow did not: retuning without a
rebuild, seeing which condition decided a run, and reporting the policy
alongside the results it produced. Selection happens once, before search
starts, so it costs nothing at run time.

A file passed as ``policy_file`` replaces the built-in table. It must contain a
``strategy`` section, a ``heuristic`` section, or both, and it is validated on
load: an unknown feature name, an unknown outcome label, or an unconditional
rule followed by further rules is an error. A policy that does not validate
does not fall back to the built-in one, since planning silently under a
different policy than the file asked for is worse than refusing to start.
