float32 time
string action
float32 duration

# --- Epistemic policy extension ---
#
# A plan for a partially observable domain is a policy, not a sequence: after a
# sensing action the executor must run one continuation or another depending on
# what was observed. These fields let a Plan carry that shape, so that the
# planner and the executor keep exchanging one message type.
#
# A classical planner leaves all of them at their defaults, which describes a
# plan read straight through from items[0] to the end. When `children` is used,
# items[0] is the root of the policy and every other item is reached only by
# being named in some item's `children`.

# The action's name in the epistemic task's own vocabulary — the grounded name
# the planner searched over, before translation into the PlanSys2 expression in
# `action`. The two are different vocabularies, and the epistemic state needs
# this one to find the action's event model and apply its update.
string epistemic_action

# Index into Plan.items of the continuation to run for each possible outcome.
# Empty: the policy ends here. One entry: an unconditional continuation, which
# is what an ontic action has. Several: a branch, one per sensing outcome.
uint32[] children

# A `children` entry equal to this means the policy is complete on that
# outcome: nothing more to run, rather than a continuation at index 4294967295.
# It is needed because "observed an outcome the policy does not cover" and
# "observed an outcome that finishes the policy" must not look alike.
uint32 POLICY_DONE = 4294967295

# The observation selecting each entry of `children`, same length and order.
# These are event names of the sensing action, and it is by matching one that
# the executor decides which continuation to run.
string[] outcomes

# True when this action is a sensing action, so its outcome must be observed
# before the policy can continue. False for ontic actions, whose effect on
# knowledge is determined by the action alone.
bool sensing false

# Epistemic conditions that must hold before this action runs, as formula
# strings over the agents' knowledge — "(K r1 (clear corridor))". These are not
# expressible as PDDL preconditions, which is why they travel here rather than
# being read from the domain.
string[] knowledge_requirements

# --- Temporal policy extension ---
#
# A temporal policy (Plan.temporal) overlaps actions, and starts each one when
# the actions it waits for have ended rather than at a clock time: with
# durations that jitter, only that keeps the belief the same as planned. `time`
# is the planned start, for display and scheduling estimates; these fields are
# what the executor dispatches by. `children` and `outcomes` are unused.

# The items this one waits for: every item on its path planned to end before
# it starts, less those another listed item already waits for. It starts once
# all of them have ended; waiting only for the latest ends would let it start
# while an earlier-planned end is still outstanding, an overlap nobody checked.
# Empty: it starts with the policy.
uint32[] starts_after

# The branch this item belongs to: it runs only if each item in `given_items`
# observed the outcome at the same position of `given_outcomes`. Every
# observation on the path from the start of the policy is listed, not only the
# latest, since an item can start after a decision point that started nothing.
# If any of these items did not run, neither does this one.
uint32[] given_items
string[] given_outcomes
