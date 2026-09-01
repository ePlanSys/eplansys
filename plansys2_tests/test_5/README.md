# Test 5

A branching policy, executed end to end: the corridor mission, run twice with
the corridor in different states.

Nobody knows whether the corridor is blocked, so there is no plan to make —
only a policy. The robot drives to the junction, looks, and reports what it
found, and which report it makes is decided while it runs.

## PlanSys2

- [x] Regular PlanSys2 actions
- [ ] Regular PlanSys2 actions with ROS2 action client
- [ ] PlanSys2 BT actions
- [ ] PlanSys2 BT actions with ROS2 action client

## Epistemic

- [x] Epistemic plan solver, returning a policy rather than a flattened branch
- [x] `EpistemicBTBuilder` loaded by the executor through pluginlib
- [x] Epistemic behavior tree nodes loaded through `bt_node_plugins`
- [x] `CheckKnowledge` against the epistemic state
- [x] `ApplyEpistemicUpdate` with the outcome a performer observed
- [x] `EpistemicSwitch` selecting a continuation
- [x] `CheckEpistemicGoal` at the end of the policy

## PDDL

- [x] Types
- [x] Predicates
- [x] Durative actions
  - [x] at start req
  - [x] over all req
  - [ ] at end req
  - [x] at start effect
  - [x] at end effect

## The mission

The epistemic half is the grounded task
`plansys2_epistemic_planner/test/tasks/robot-fleet.json`: two agents, an atom
for the corridor being blocked, and an initial model that designates both a
blocked and a clear world — the corridor genuinely is one or the other, and no
one knows which. `inspect` is semi-private sensing: `r1` sees the outcome,
`r2` only sees that `r1` looked. The goal is that both know whether the
corridor is blocked, which is why `r1` has to announce what it found.

The PDDL half, `pddl/test_5.pddl`, is only what the executor needs to drive
the four actions the mapping translates into. It records nothing about the
corridor: a predicate for "blocked" would be recording knowledge as a fact,
which is the thing the epistemic state exists to avoid.

## What stands in for a sensor

The epistemic state can name the outcome of a sensing action only when its
model designates a single world. Here it designates two, so the observation
must come from the agent that performed the sensing: the performer running
`inspect_corridor`.

It reports the result the way any performer reports anything, on finishing, in
the `outcome` field of `ActionExecution` alongside its status. `ExecuteAction`
writes it to the blackboard and the packaged action template passes it to
`ApplyEpistemicUpdate`, so this mission runs on the executor's defaults with no
action template and no additional node library.

`TestAction::set_outcome` stands in for a sensor reading. Setting it to
`e-inspect-clear` or `e-inspect-blocked` is the only difference between the two
runs, which must execute different actions.

Test 6 binds `observed` explicitly, the approach required when the observation
originates elsewhere than the acting performer.
