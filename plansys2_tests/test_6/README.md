# Test 6

A policy that branches twice, executed end to end: the depot mission, run with
the two corridors in opposite states.

Three robots, two corridors, and no one knows the state of either. Knowing
about one says nothing about the other, so the policy senses twice, and the
second sensing happens inside whichever continuation the first one selected.
Four ways the depot can turn out, four paths through one tree.

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
- [x] `EpistemicSwitch` nested inside another switch's continuation
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

`plansys2_epistemic_planner/test/tasks/robot-fleet-depot.json`: three agents,
an atom per corridor, and an initial model designating all four combinations of
blocked and clear. The goal is that all three robots know the state of both
corridors, which takes two sensings and two announcements — the announcements
being how the robots that did not look find out.

The policy AO* returns has twelve nodes and a switch under a switch. Each run
here follows one path through it, and the two runs share nothing after the
first observation.

## What stands in for a sensor

As in test_5, the outcome of a sensing action has to come from whoever did the
sensing, because the model designates more than one world and cannot choose
between them. The observations are reported per action — the depot has two
things to find out and they need not agree, which is exactly what a single
mission-wide observation could not express.
