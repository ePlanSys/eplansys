# Test 7

The corridor mission against a world that does not behave as the policy
expected. Tests 5 and 6 check that the right branch runs; this one checks that
no planned action runs on a belief the robot's own observation has just
contradicted.

## PlanSys2

- [x] Regular PlanSys2 actions
- [ ] Regular PlanSys2 actions with ROS2 action client
- [ ] PlanSys2 BT actions
- [ ] PlanSys2 BT actions with ROS2 action client

## Epistemic

- [x] Epistemic plan solver, returning a policy
- [x] `EpistemicBTBuilder` loaded by the executor through pluginlib
- [x] `ApplyEpistemicUpdate` refusing an observation the model cannot represent
- [x] `ApplyEpistemicUpdate` repairing the model to accept an outcome it held
      impossible
- [x] `CheckKnowledge` refusing the planned action against the repaired model

## The two runs

Both reuse test 5's PDDL and the `robot-fleet.json` action mapping.

**An observation the model cannot account for.** The task is
`robot-fleet.json`, and the policy branches on whether the corridor is clear or
blocked. The inspecting performer reports `e-inspect-foggy`, which no event of
`inspect` describes. The epistemic state refuses the update, and the mission
fails after the inspection, before either report.

**An outcome the policy did not plan for.** The task is
`robot-fleet-stale-prior.json`: the robots are at the junction and the map says
the corridor is clear, so the model designates only the clear world, and the
planner, sure of the answer, plans for that one outcome. The performer reports
`e-inspect-blocked`. The state repairs the model to fit the observation, and
`report_clear`, the action planned for a clear corridor, is refused by its
knowledge guard, that `r1` knows the corridor is clear, which no longer holds. The mission
fails, and the model it leaves behind, the one a replan would start from, has
the corridor blocked and `r1` knowing it.

## Why neither run reaches `EpistemicSwitch`

The switch refuses an outcome it has no branch for. With the fleet tasks that
cannot happen end to end: `inspect` has two outcomes, so a policy that branches
on it plans for both, and one sure of the answer has a single continuation and
renders no switch at all. The switch's refusal is unit tested in
`plansys2_epistemic_executor` (`TheSwitchRefusesAnUnplannedOutcome`); this test
covers the property that refusal protects, from the other two nodes that
enforce it.
