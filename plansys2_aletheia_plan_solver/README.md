# plansys2_aletheia_plan_solver

Runs the Aletheia epistemic planner as an external process.

`plansys2_epistemic_planner` performs the same search in process and is the
better default: no fork, no serialisation, and no dependency on a binary being
installed. This package is for the cases where that is not what is wanted — a
planner built and versioned separately from the workspace, one run under its
own resource limits, or one being compared against the in-process build. Both
are `PlanSolverBase` plugins, so choosing between them is a parameter change.

It is to `plansys2_epistemic_planner` what `plansys2_popf_plan_solver` is to
POPF: the adapter, not the planner.

## What it does

```
problem string or task_file  ──>  task.json
                                     │
                                     ▼
        epistemic_planner --task task.json --plan plan.json ...
                                     │
                                     ▼
                                 plan.json  ──>  plansys2_msgs/Plan
```

The planner writes one of two shapes. AO* writes a policy tree:

```json
{"action": "peek_A",
 "branches": [{"event": 0, "subtree": {"action": "open_A", "branches": []}},
              {"event": 1, "subtree": null}]}
```

GBFS and EHC write a flat array of grounded action names. Both are read; a file
holding `null` is an empty plan, and a missing file is the planner reporting no
solution, which it does by exiting zero without writing one.

The plan file names actions and *event indices*. What a branch is taken on,
what each action requires to be known, and what the goal is are properties of
the task, not of the plan, so the task is parsed here as well and the
conversion is handed to `plansys2_epistemic_planner`'s own
`to_policy_plan`. Everything downstream of the subprocess — the parser, the
validator, the action mapping, the policy serialisation — is that package's
code. A second implementation of the same translation would be a second thing
to keep correct.

The plan is validated against the task as parsed here, in addition to the
planner's own validation. The two can disagree only if the binary and the
workspace were built from different sources, which is exactly the failure mode
a separately built planner introduces.

## Parameters

| parameter | default | meaning |
| --- | --- | --- |
| `command` | `epistemic_planner` | the binary; an absolute path when it is not on `PATH` |
| `arguments` | empty | extra arguments, appended verbatim |
| `output_dir` | system temp dir | where `task.json`, `plan.json` and `aletheia.log` are written; a leading `~` is expanded |
| `task_file` | empty | grounded task JSON, when the `problem` string is not one |
| `heuristic` | empty | `ug`, `ed`, `ks`, `wc`, `rpg`, `radd`; empty leaves it to the planner |
| `strategy` | empty | `gbfs`, `ehc`, `aostar`; empty as above |
| `policy_file` | empty | selection-policy JSON overriding the planner's built-in rules |
| `action_mapping` | empty | JSON map from grounded names to PlanSys2 action expressions |
| `conditional_plan` | `flatten` | `policy`, `flatten` or `reject` |

The last six are the in-process plugin's parameters under the same names, so a
parameters file moves between the two plugins by changing `plugin:` alone.

```yaml
planner:
  ros__parameters:
    plan_solver_plugins: ["ALETHEIA"]
    ALETHEIA:
      plugin: "plansys2/AletheiaPlanSolver"
      command: "/opt/aletheia/build/epistemic_planner"
      action_mapping: "/abs/path/to/mapping.json"
      conditional_plan: "policy"
```

## Where the output goes

`PlanSolverBase::execute_planner` captures the child's stdout, so
`aletheia.log` holds the planner's search trace rather than the plan. Its
stderr is not redirected and reaches the planner node's console. The task, the
plan and the log are left in `output_dir` after a run, which is what makes a
failure reproducible by hand with the command line the error message prints.

## What is tested

`read_plan_file` is a free function precisely so it can be tested without a
lifecycle node, and the tests pin it against the shapes the planner writes: a
chain, a branching policy, a linear plan, an empty plan, a missing file, and
three malformed files. That is the contract with a binary this package does not
build.

Not covered: the subprocess itself, which needs the planner installed, and the
plugin inside a running planner node.
