# plansys2_epddl_grounder

The EPDDL front end. It turns an EPDDL domain, problem and action-type
libraries into the grounded task JSON that `plansys2_epistemic_planner` searches
over and that the epistemic state tracks, so that a launch file can be given
`.epddl` sources the way classical PlanSys2 is given a `.pddl` domain.

## Why a subprocess

Grounding EPDDL is not parsing. The initial state of a problem is written as a
finitary S5 theory, and grounding it means *constructing* the Kripke model that
theory describes — four worlds and two accessibility relations, for two muddy
children — and then instantiating each action's event model over the object
universe. That work belongs to
[plank](https://github.com/HanielUlises/plank), the EPDDL toolkit, and this
package runs it rather than reimplementing it:

```bash
plank export -d <domain.epddl> -p <problem.epddl> -l <library.epddl>... -o <dir>
```

plank is listed in `dependency_repos.repos`, so building the workspace builds it
and sourcing the workspace puts it on PATH. Nothing here links against it: a
workspace built without plank still builds and its tests still pass, and only
the EPDDL route is unavailable — which the grounder reports, with the way to fix
it, rather than failing obscurely.

## Using it

From a node, through parameters — the planner plugin prefixes them with its
plugin name, the epistemic state node does not:

| parameter | meaning |
| --- | --- |
| `epddl_domain` | path to the EPDDL domain |
| `epddl_problem` | path to the EPDDL problem |
| `epddl_libraries` | action-type libraries the domain declares; empty uses the packaged `intermediate` |
| `plank_command` | path to plank; empty uses `$PLANK`, then PATH |

By hand, to inspect a task or check one into a test:

```bash
ros2 run plansys2_epddl_grounder ground_epddl \
  -d $(ros2 pkg prefix plansys2_epddl_grounder)/share/plansys2_epddl_grounder/examples/muddy-children-domain.epddl \
  -p $(ros2 pkg prefix plansys2_epddl_grounder)/share/plansys2_epddl_grounder/examples/muddy-children-problem.epddl \
  -o /tmp/muddy-children.json
```

That command reproduces `plansys2_epistemic_planner/test/tasks/muddy-children-2.json`
byte for byte, which is what ties the front end to the fixtures the planner's
own tests are pinned against.

## Grounding twice

The planner asks for a task on every `get_plan`, so the grounder caches: the
sources are ground on first use and again only when a path or a modification
time changes. Editing a domain and re-planning re-grounds; planning twice does
not fork plank twice.

## Failure

plank reports a bad specification by printing the offending line of the
specification and then dying on a signal rather than exiting with a status. The
grounder therefore judges success by whether the task file was written, and
passes plank's own output through as the error — the syntax error is in there,
and it is the thing a user needs to see.

## What is packaged here

`libraries/intermediate.epddl` is a verbatim copy of plank's `intermediate`
action-type library, and `examples/` holds the muddy-children EPDDL sources.
Both are installed under `share/plansys2_epddl_grounder`, since they are read at
run time by path. See the repository `NOTICE` for their provenance.
