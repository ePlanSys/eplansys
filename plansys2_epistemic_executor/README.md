# plansys2_epistemic_executor

Executing plans for agents that do not know everything.

PlanSys2 executes a plan as a behavior tree: a sequence of per-action subtrees,
each guarding its action with the PDDL conditions around it. That works because
a classical plan commits to one future — every action's preconditions are known
to hold when its turn comes, and there is nothing to decide at run time.

A plan for a partially observable domain is not a sequence. After a sensing
action, which action comes next depends on what was observed, and some
preconditions are not facts at all but statements about what an agent knows.
This package executes those plans, by extending PlanSys2's tree rather than
replacing it.

## What a policy is

The planner returns a tree, and it travels in the epistemic fields of
`plansys2_msgs/PlanItem`:

- `children` — one continuation per possible outcome, by item index
- `outcomes` — the observation that selects each, in the same order
- `sensing` — whether the outcome has to be observed at all
- `knowledge_requirements` — the epistemic conditions the action needs
- `epistemic_action` — the action's name in the planner's own vocabulary

A classical plan sets none of them and is read the way PlanSys2 writes it: item
`i` followed by item `i+1`. So every plan is a policy here, and the distinction
that matters is whether it branches.

## The tree

Each policy node renders as the PlanSys2 action subtree — unchanged, driving
the same action performers against the same problem expert — wrapped in the
three things a sequence cannot express:

```
Sequence "node_0 (peek A)"
  CheckKnowledge         node="0"          <- what must be known before acting
  WaitAtStartReq         action="(peek A):0"           ┐
  ApplyAtStartEffect     action="(peek A):0"           │
  ReactiveSequence                                     │ PlanSys2's own
    CheckOverAllReq      action="(peek A):0"           │ nodes, driving the
    ExecuteAction        action="(peek A):0"           │ same performers
                         outcome="{epistemic_observed_0}"
  CheckAtEndReq          action="(peek A):0"           │
  ApplyAtEndEffect       action="(peek A):0"           ┘
  ApplyEpistemicUpdate   node="0" observed="{epistemic_observed_0}"
                                  outcome="{epistemic_outcome_0}"
  EpistemicSwitch        node="0" outcome="{epistemic_outcome_0}"
                                  outcomes="e_tails;e_heads"
    Sequence "node_1 (shout-tails A)"     <- observed e_tails
    Sequence "node_2 (open A)"            <- observed e_heads
CheckEpistemicGoal goal="(K A tails)"
```

**CheckKnowledge** is the counterpart of `CheckOverAllReq` for conditions the
problem expert cannot answer. "The corridor is clear" is a fact and lives
there. "r1 knows whether the corridor is clear" is not a fact about the
corridor; no set of predicates records it, and a plan that senses before
committing turns on exactly that difference.

**ApplyEpistemicUpdate** is the counterpart of `ApplyAtEndEffect`, one level
up. An action changes what agents know by more than its own effects: an
announcement informs everyone listening, and an observation that finds nothing
still rules worlds out. This performs the DEL product update, so the next
action's guard is checked against the state this action produced.

**EpistemicSwitch** has no counterpart at all. It reads the outcome the update
reported and runs the continuation planned for it. An outcome the policy does
not list fails the node rather than defaulting to a branch — every branch was
built for a different belief, and running one anyway is a robot acting
confidently on a belief nothing supports. The failure reaches the executor,
whose answer to a failed plan is to replan.

**CheckBeliefUnchanged** fails when the belief moves for a reason the policy
did not account for. Its own actions are not such a reason; an announcement is
— perception resolving a region, an operator saying something, two robots
reconciling their maps. The packaged template puts it inside the reactive
sequence, so the interruption arrives while the invalidated action is still
running rather than after it. The executor replans, from the model the
announcement produced.

**CheckEpistemicGoal** runs once, after the policy. Reaching a leaf means one
execution finished, not that the goal holds: every leaf was believed to reach
it, but that belief was formed against the model at planning time.

A node with a single continuation renders without a switch, so a classical plan
comes out as the same flat sequence PlanSys2 would have built.

## The epistemic state

`epistemic_state_node` is to knowledge what the problem expert is to facts. It
holds a pointed Kripke model and answers three things:

| service | question |
| --- | --- |
| `epistemic_state/load_task` | be the model of *this* task — a grounded task inline or by path, or EPDDL sources to ground |
| `epistemic_state/check_formula` | does `(K r1 (clear corridor))` hold now? |
| `epistemic_state/apply_action` | this action ran: update, and say what was observed |
| `epistemic_state/get_goal` | what are we aiming at, and does it hold yet? |
| `epistemic_state/set_goal` | aim at this instead, without re-grounding the problem |
| `epistemic_state/announce` | everyone just learned this is true |

The last three are what make it the problem expert's counterpart rather than
only a service the executor calls. Announcing is `set predicate` one level up,
and not the same operation: setting a predicate changes what is true,
announcing changes what is *known*. Announcing `muddy_c1` in the muddy-children
model makes `(K c1 muddy_c1)` hold when it did not, because it rules out the
worlds `c1` could not previously tell the real one from.

From a shell, all of it is one verb:

```bash
ros2 plansys2 epistemic show
ros2 plansys2 epistemic check "(Kw c1 muddy_c1)"
ros2 plansys2 epistemic goal "(K A tails)"
ros2 plansys2 epistemic announce "tails"
ros2 plansys2 epistemic apply ask_c1
```

It lives in `plansys2_tui_cli` rather than in `plansys2_terminal`: that package
is built by every distribution's workflow, and reaching these services from C++
would make it depend on the epistemic messages to do so.

It advances by executed actions rather than by watching the world, which is
what makes it a belief state rather than a log. When it disagrees with what the
robot observed, the disagreement surfaces at `apply_action` as an outcome the
model cannot account for — a reason to replan, not to overwrite the model
quietly.

Which outcome occurred is a question about the world, and the state can answer
it alone only when it designates a single world. With several designated the
model is undetermined, and the observation must come from the agent that
performed the sensing.

It arrives from the performer along the path every other execution report
takes. `ActionExecution` carries an `outcome` alongside `status`, which a
performer sets on finishing:

```cpp
finish(true, 1.0, "Corridor inspected", "e_clear");
```

The tree then forwards it: `ExecuteAction` writes it to the blackboard and the
packaged template binds that entry to `ApplyEpistemicUpdate`'s `observed` port.
A sensing action therefore requires no configuration beyond a performer that
reports its result. The token must name an outcome of the action's model, since
the following switch branches on it; `status` remains the human-readable log
message, and a classical performer that reports nothing is unaffected.

Binding `observed` explicitly remains the way to supply an observation that
originates elsewhere than the acting performer, such as an independent
perception node or a test fixture.

## How it plugs in

Two parameters on the executor, and nothing else:

```yaml
executor:
  ros__parameters:
    bt_builder_plugin: "EpistemicBTBuilder"
    bt_node_plugins: ["libplansys2_epistemic_bt_nodes.so"]
```

The state node is not started by hand either. Both bringup launch files start
it on `epistemic_state:=True` and hand it to the lifecycle manager with the
other four, so it comes up and goes down with the system. Its problem goes in
the same parameters file:

```yaml
epistemic_state:
  ros__parameters:
    epddl_domain: "/abs/path/to/domain.epddl"
    epddl_problem: "/abs/path/to/problem.epddl"
```

Those are the same two EPDDL files the solver is given. The state grounds them
itself rather than being handed the planner's task, because a service call at
configure time would make the two nodes depend on each other's start-up order.

It stays a separate process even under the monolithic launch, and
`plansys2_node` manages it through `LifecycleServiceClient`, which needs its
name and not its class. That is the same boundary as `bt_node_plugins`: nothing
in `plansys2_bringup` links against this package.

`bt_node_plugins` is a generic hook: the executor loads BehaviorTree.CPP node
libraries before building a tree. That is what keeps `plansys2_executor` free
of any dependency on this package — it never links against the epistemic nodes,
it loads them. The nodes find the policy on the blackboard, where the executor
already publishes the plan it is running.

## The packages

| package | what it is | depends on plansys2_executor |
| --- | --- | --- |
| `plansys2_epistemic_msgs` | the three service definitions | no |
| `plansys2_epistemic_executor` | policy, tree rendering, the four nodes, the state | no |
| `plansys2_epistemic_bt_builder` | the `BTBuilder` plugin | yes |

The builder is a separate package because it is the only piece that needs the
executor's plugin API, and keeping it apart lets everything else build and be
tested against a released distribution rather than only against this fork.

## What is tested, and what is not

The policy reading, the tree rendering, and the formula round trip between the
planner and the state are covered by tests that run anywhere — including the
check that every rendered tree parses in the real BehaviorTree.CPP.

Not covered: the four nodes ticking against a live state node, and the builder
inside a running executor. Both need a full PlanSys2 stack up, which this
fork's rolling sources cannot build on Humble; they are exercised in CI on
rolling.
