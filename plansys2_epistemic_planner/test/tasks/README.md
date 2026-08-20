# Test tasks

Grounded epistemic planning tasks in the IePC JSON format — the same format
`EpistemicPlanSolver` accepts, either as its `task_file` parameter or inline as
the `problem` string of `planner/get_plan`.

All but one were produced by `plank ground` from the EPDDL sources in the
[Epistemic-Robotics](https://github.com/HanielUlises/epistemic-robotics)
`epddl-workspace/`, and are checked in verbatim so the tests pin the parser
against the grounder's own accounting (`planning-task-info` carries the agent,
atom, action, and world counts the tests assert).

| file | source | why it is here |
| --- | --- | --- |
| `muddy-children-2.json` | `muddy-children/out/muddy-children-problem.json` | smallest solvable task; the default for search tests |
| `muddy-children-3.json` | `muddy-children/out/muddy-children-problem-3.json` | asymmetric variant, used for the determinism check |
| `coin-in-the-box.json` | `coin-in-the-box/out/problem_1.json` | sensing domain with a `box` (not `Kw`) goal |
| `active-muddy-child.json` | `Active-Muddy-Child/out/problem_1.json` | 32 initial worlds; the canonical partial-observability case |
| `coin-in-the-box-multipointed.json` | derived, see below | the only fixture whose solution is a branching policy |

## The derived fixture

Every task plank grounds from these instances is single-pointed — one
designated world — so sensing has exactly one possible outcome and AO* returns
a chain. Nothing in the workspace exercises contingent branching, which is
precisely the path `EpistemicPlanSolver` flattens away when it converts a
policy into a `plansys2_msgs/Plan`.

`coin-in-the-box-multipointed.json` is `coin-in-the-box.json` with two edits:

- `initial-state.designated` is `["w0", "w1"]` rather than `["w1"]`, so it is
  genuinely open whether the coin lies tails;
- the goal is `Kw_A(tails)` rather than `K_A(tails)`, since knowing *that* the
  coin is tails is unachievable in the world where it is not.

That is the textbook multi-pointed coin-in-the-box, and its solution branches.
If the EPDDL instance is ever written and grounded, replace this file with the
grounder's output.
