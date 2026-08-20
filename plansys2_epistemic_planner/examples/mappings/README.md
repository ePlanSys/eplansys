# Action mappings

Aletheia's action names and PlanSys2's are two vocabularies, not two spellings
of one. `plank ground` turns an EPDDL action into a single token —
`pickup-A-hold_r2`, `observe-private-A_r1`, `ask_c1` — while the executor takes
`(pickup r2 A)`, splits it into a name and parameters, and looks the name up in
the PDDL domain to find the behavior tree that drives the hardware. An
untranslated name matches nothing there, so the robot does not move.

A mapping file states the correspondence. Point the solver at one with the
`action_mapping` parameter:

```yaml
planner:
  ros__parameters:
    EPISTEMIC:
      action_mapping: "/abs/path/to/mapping.json"
```

## Format

A JSON object keyed by grounded action name. A value is either the action
expression alone, or an object that also carries a duration in seconds:

```json
{
  "ask_c1":          "(ask c1)",
  "move-kitchen_r1": {"action": "(move r1 corridor kitchen)", "duration": 12.5}
}
```

Duration defaults to 1.0 and must be positive. It is a property of the robot's
action implementation rather than of the plan — the epistemic planner is
untimed — and it is what the executor schedules and times out against, so
unit durations are worth replacing before running on hardware.

An action the map does not cover fails the planning request. It does not fall
back to the convention below: a guessed name the domain does not declare would
reach the executor as a plan it cannot dispatch, and the reason would surface
far from the mapping that caused it.

## The fallback convention

With `action_mapping` unset, the solver splits the grounded name at its first
underscore and reads the rest as parameters, keeping hyphens in the name:

| grounded | convention |
| --- | --- |
| `ask_c1` | `(ask c1)` |
| `signal_A_B` | `(signal A B)` |
| `pickup-A-hold_r2` | `(pickup-A-hold r2)` |

It is enough to see a plan come out in the right shape, and it is what the last
row shows the limit of: the grounded name does not record which parameter order
the PDDL domain declares, and no convention can recover it. Write the map
before dispatching to real actions.
