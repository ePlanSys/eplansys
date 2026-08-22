# plansys2_epistemic_perception

The map, read as knowledge.

The epistemic state is advanced by executed actions and not by observing the
world. That is on purpose: it holds what the agents know, and knowledge changes
by events with an event model behind them, not by whatever arrives on a topic.
But a policy that senses has to be told what was sensed, and nothing in
PlanSys2 or in ePlanSys was looking at a map. This package is what closes that
loop.

It watches an `nav_msgs/OccupancyGrid`, decides what named regions of it are,
and says so to the epistemic state in the vocabulary of the loaded task.

## Two things stand between a grid and a formula

**The quantifier.** A cell is free or occupied; a proposition is about a
region. Which quantifier bridges them is a modelling decision, and it is taken
in one place:

| The region is | when |
| --- | --- |
| `Clear` | every cell in it has been observed and settled as free |
| `Blocked` | any one cell in it is occupied |
| `Unknown` | anything else |

Occupied beats unobserved, because no further looking makes an obstacle go
away. Unobserved beats free, because a region is only clear when all of it is.
A cell that has been seen without being settled — the band between the two
thresholds — counts as unobserved, since calling it free would hand an agent
knowledge it does not have.

**The vocabulary.** A region is a place on a map; an atom is a name in a
grounded task, and a grounded task's names are flat: `blocked`,
`at-junction_r1`. A region therefore says which atom its occupancy decides and
which way that atom points. `blocked` with `atom_true_when_clear: false` is the
corridor of the fleet tutorial. `clear_corridor` with it true is the same thing
written the other way, and both occur in real domains.

## How it reports depends on why

```
                    region bound to a sensing action?
                     /                          \
                   yes                          no
                    |                            |
   ApplyAction(action, outcome)          Announce(formula)
   the observation belongs to an         information from outside the
   action the planner branched on        plan: an operator, or two robots
                                         reconciling their maps
```

A sensing action in a policy does not decide its own outcome — the planner
enumerated the outcomes and the robot has to say which occurred. When that
observation is a look at the map, the binding turns it into the event name the
state expects. Without a binding, the same observation goes out as a public
announcement, which is what the state offers for something everyone witnessed.

It reports on change, not on every grid. A map arrives several times a second
and says the same thing each time; announcing a formula twice is harmless in
the model, but applying a sensing action twice is not, and both routes follow
the same rule.

## Configuring it

```yaml
epistemic_perception:
  ros__parameters:
    map_topic: "/map"
    regions: ["corridor"]
    corridor:
      boxes: [2.0, -0.5, 8.0, 0.5, 7.5, 0.5, 8.5, 4.0]
      atom: "blocked"
      atom_true_when_clear: false
      sensing_action: "inspect_r1"
      outcome_when_clear: "e-inspect-clear"
      outcome_when_blocked: "e-inspect-blocked"
```

| Parameter | Default | Meaning |
| --- | --- | --- |
| `map_topic` | `/map` | The grid to watch. Subscribed transient-local, so a map published before this node came up still counts. |
| `regions` | empty | The regions to watch. Each one's settings are declared under its own name. |
| `<region>.boxes` | — | `[min_x, min_y, max_x, max_y]` per box, in metres in the map frame. Their union is the region. Required. |
| `<region>.atom` | `<predicate>_<region>` | The atom this region's occupancy decides. |
| `<region>.predicate` | `clear` | Only used to build the default atom name. |
| `<region>.atom_true_when_clear` | `true` | Whether the atom asserts that the region is clear, or that it is not. |
| `<region>.sensing_action` | empty | The grounded action this observation answers. Empty means announce instead. |
| `<region>.outcome_when_clear` | empty | The event that fired, when the region turned out clear. Required with a sensing action. |
| `<region>.outcome_when_blocked` | empty | The same, for blocked. |
| `free_below` | `25` | Below this, a cell is free enough to drive. |
| `occupied_above` | `65` | Above this, a cell is an obstacle. |
| `call_timeout` | `5.0` | Seconds to wait for the state to answer one call. |

Regions are given in metres rather than in cells because a map is re-gridded as
it grows: slam_toolbox moves the origin and changes the width, and a region
written in cell indices would quietly come to mean somewhere else.

## What it does not do

It does not track who observed what. Everything it reports is either an
outcome of an action a named agent executed, or a public announcement to all of
them; per-agent coverage — the difference between a robot having seen a cell
and having been handed a map that contains it — is a separate question, and the
model needs an event model per observer to express it.

It does not retract. A region that goes back to undecided, because the map grew
into somewhere nobody has been, produces no call: the state has no operation
for taking knowledge back, and pretending otherwise would put the model and the
plan out of step silently.
