plansys2_msgs/PlanItem[] items

# --- Epistemic policy extension ---
#
# The goal the policy was built to achieve, as an epistemic formula string.
# Empty for a classical plan, whose goal is the PDDL goal held by the problem
# expert. It travels with the policy because reaching every leaf of a policy is
# not the same as achieving the goal: the executor checks it once execution
# settles on a leaf.
string epistemic_goal

# True for a temporal policy: items overlap, and each starts when the items in
# its `starts_after` have ended, on the branch its `given_items` select. The
# planner sets it only for a policy whose overlapping actions it has proven to
# commute, so the order in which they end does not change the belief.
bool temporal false
