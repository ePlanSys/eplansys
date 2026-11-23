# Test 8

A temporal policy, executed end to end: the corridor mission planned with
durations.

With durations, the corridor's best policy is not the shortest one. Sending
both robots to look at once takes 30 s (drive) + 5 s (look) = 35 s; one robot
looking and telling the other takes 30 + 5 + 1 = 36 s, though it is one step
shorter. The planner (`EPISTEMIC.temporal: true`) returns the 35 s policy, a
temporal one: the two drives overlap, and so do the two looks, because their
updates commute in every state the running set can pass through.

## Epistemic

- [x] Temporal planning with the action mapping's durations
- [x] A temporal `plansys2_msgs/Plan` (`temporal`, `starts_after`, `given_*`)
- [x] `EpistemicBTBuilder` rendering it for event-driven dispatch
- [x] `WaitForEnds`, `CheckGiven`, `MarkDone` gating every item
- [x] Two sensing actions running at once against one epistemic state
- [x] `CheckEpistemicGoal` at the end

## What it checks

Both robots drive and both look, nobody reports, the mission succeeds, and the
executor's own timestamps show the drives overlapping and the looks
overlapping. It runs twice, once per corridor state.

A performer serves one request at a time, so there are two performers for each
action that overlaps with itself. The classical half is test 5's PDDL.
