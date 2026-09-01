Architecture
============

ePlanSys adds packages to PlanSys2 and changes none of its own. The planner is
a plan solver plugin, the tree builder is a behavior tree builder plugin, the
epistemic behavior tree nodes are loaded as a BehaviorTree.CPP plugin library,
the model of what the agents know is a separate lifecycle node, and perception
is a second one that watches a map and reports to the first. Around them sit a
front end that grounds EPDDL into the task they all read, a command line that
asks the model questions, and a metapackage that installs the set. Nothing in
``plansys2_executor`` links against any of it.

.. graphviz::
   :caption: Where the epistemic packages attach to PlanSys2

   digraph architecture {
     rankdir=LR;
     node [shape=box, fontname="sans-serif", fontsize=10];
     edge [fontname="sans-serif", fontsize=9];

     subgraph cluster_plansys2 {
       label="PlanSys2";
       fontname="sans-serif";
       fontsize=10;
       planner   [label="planner"];
       executor  [label="executor"];
       problem   [label="problem expert"];
       domain    [label="domain expert"];
     }

     subgraph cluster_epistemic {
       label="ePlanSys";
       fontname="sans-serif";
       fontsize=10;
       solver    [label="EpistemicPlanSolver\n(plan solver plugin)"];
       builder   [label="EpistemicBTBuilder\n(BT builder plugin)"];
       nodes     [label="epistemic BT nodes\n(BT.CPP plugin library)"];
       state     [label="epistemic_state\n(lifecycle node)"];
       perceive  [label="epistemic_perception\n(lifecycle node)"];
     }

     map [label="occupancy grid\n(SLAM)", shape=ellipse];

     planner  -> solver   [label="loads"];
     executor -> builder  [label="loads"];
     executor -> nodes    [label="loads"];
     executor -> problem  [label="facts"];
     executor -> domain   [label="action trees"];
     nodes    -> state    [label="services"];
     map      -> perceive [label="grid"];
     perceive -> state    [label="outcomes,\nannouncements"];
     solver   -> executor [label="Plan with\nepistemic fields", style=dashed];
   }

Plan solver plugin
------------------

``plansys2/EpistemicPlanSolver`` implements the ``plansys2::PlanSolverBase``
interface and is exported to ``plansys2_core`` by the plugin description in
``plansys2_epistemic_planner``. It is selected by listing it in
``plan_solver_plugins`` and naming its class under that entry:

.. code-block:: yaml

   planner:
     ros__parameters:
       plan_solver_plugins: ["EPISTEMIC"]
       EPISTEMIC:
         plugin: "plansys2/EpistemicPlanSolver"

Its parameters are declared under the plugin's own name at configure time, and
are strings except where noted:

.. list-table::
   :header-rows: 1
   :widths: 22 18 60

   * - Parameter
     - Default
     - Meaning
   * - ``epddl_domain``
     - empty
     - Path to the EPDDL domain. With ``epddl_problem``, the ordinary way to
       state the problem, and the epistemic counterpart of ``model_file``.
   * - ``epddl_problem``
     - empty
     - Path to the EPDDL problem.
   * - ``epddl_libraries``
     - empty
     - Action-type libraries the domain declares. Empty supplies the
       ``intermediate`` library packaged with ``plansys2_epddl_grounder``.
       This one is a string array; the rest are strings.
   * - ``plank_command``
     - empty
     - Path to the plank binary. Empty takes ``$PLANK``, then PATH.
   * - ``task_file``
     - empty
     - Absolute path to an already grounded task JSON. Ignored when the EPDDL
       sources are set; with neither, the task must arrive in the planning
       request instead.
   * - ``heuristic``
     - empty
     - Pins a heuristic: ``ug``, ``ed``, ``ks``, ``wc``, ``rpg`` or ``radd``.
       Empty leaves the choice to the selection policy.
   * - ``strategy``
     - empty
     - Pins a search strategy: ``gbfs``, ``ehc`` or ``aostar``. Empty leaves
       the choice to the selection policy.
   * - ``policy_file``
     - empty
     - Path to a JSON selection policy replacing the built-in rule table.
   * - ``action_mapping``
     - empty
     - Path to a JSON map from grounded epistemic action names to PlanSys2
       action expressions. Empty falls back to the naming convention.
   * - ``conditional_plan``
     - ``flatten``
     - What to do with a branching solution: ``policy``, ``flatten`` or
       ``reject``. The parameters file shipped by ``plansys2_bringup`` sets
       ``policy``.

The solver validates a solution before returning it, and a plan that fails
validation is not returned at all.

Running the planner as a subprocess
-----------------------------------

``plansys2_aletheia_plan_solver`` provides a second plan solver plugin,
``plansys2/AletheiaPlanSolver``, which reaches the same planner by running its
binary rather than by linking it. It is to the epistemic planner what
``plansys2_popf_plan_solver`` is to POPF: an adapter that writes the input to a
file, runs a process under the solver timeout, and reads its output back.

.. code-block:: yaml

   planner:
     ros__parameters:
       plan_solver_plugins: ["ALETHEIA"]
       ALETHEIA:
         plugin: "plansys2/AletheiaPlanSolver"
         command: "/abs/path/to/epistemic_planner"
         conditional_plan: "policy"

It adds three parameters to the six above: ``command``, the binary, which
defaults to ``epistemic_planner`` on the path; ``arguments``, appended
verbatim; and ``output_dir``, where the task, the plan and the planner's log
are written. The other six keep their names and meanings, so a parameters file
moves between the two plugins by changing ``plugin`` alone.

The plan file names actions and event indices only. What a branch is taken on,
what an action requires to be known, and what the goal is are properties of the
task rather than of the plan, so the task is parsed on this side as well and
the conversion is performed by ``plansys2_epistemic_planner``'s own policy
serialisation. The returned plan is also validated against the task as parsed
here, in addition to the planner's own validation: the two can disagree only if
the binary and the workspace were built from different sources, which is the
failure a separately built planner introduces.

The in-process plugin remains the better default, since it costs no process
launch, no serialisation, and no separately installed binary. The subprocess
plugin is for a planner versioned apart from the workspace, one run under its
own resource limits, or one being compared against the in-process build.

Policies
--------

A policy is a tree, and it travels in the epistemic fields of
``plansys2_msgs/PlanItem``:

.. list-table::
   :header-rows: 1
   :widths: 30 70

   * - Field
     - Contents
   * - ``children``
     - One continuation per possible outcome, by item index.
   * - ``outcomes``
     - The observation selecting each, in the same order.
   * - ``sensing``
     - Whether the outcome has to be observed at all.
   * - ``knowledge_requirements``
     - The epistemic conditions the action needs.
   * - ``epistemic_action``
     - The action's name in the planner's own vocabulary.

A classical plan sets none of them and is read the way PlanSys2 writes it: item
``i`` followed by item ``i+1``. Every plan is therefore a policy here, and the
distinction that matters is whether it branches.

Behavior tree
-------------

``plansys2::EpistemicBTBuilder`` renders each policy node as the PlanSys2
action subtree, unchanged, wrapped in the three things a sequence cannot
express: a guard on what must be known, the knowledge update the action
performs, and a branch on what was observed.

.. code-block:: text

   Sequence "node_0 (peek A)"
     CheckKnowledge         node="0"
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
       Sequence "node_1 (shout-tails A)"
       Sequence "node_2 (open A)"
   CheckEpistemicGoal goal="(K A tails)"

A node with a single continuation renders without a switch, so a classical plan
comes out as the same flat sequence PlanSys2 would have built.

Reporting an observation
------------------------

Which outcome a sensing action produced is a question about the world, and the
epistemic state can answer it alone only when its model designates a single
world. With several designated the model is undetermined, and the answer must
come from the agent that performed the sensing.

The observation travels on the protocol that carries every other execution
report. ``plansys2_msgs/ActionExecution`` defines an ``outcome`` field
alongside ``status``, meaningful on ``FINISH``, which a performer sets when it
finishes:

.. code-block:: cpp

   // in the performer's do_work, once the sensor has answered
   finish(true, 1.0, "Corridor inspected", clear ? "e-inspect-clear"
                                                 : "e-inspect-blocked");

The token must name an event of the action's model, since the policy branches
on it. ``status`` remains the human-readable message for the log. An ordinary
action leaves the field empty, which is the default for every classical
performer and the reason none of them required changes.

The value then reaches the tree without further configuration.
``ExecuteAction`` provides an ``outcome`` output port, the packaged action
template binds it to ``ApplyEpistemicUpdate``'s ``observed`` input, and each
policy node receives its own blackboard entry, so two sensing actions on
different branches cannot overwrite each other.
``plansys2_msgs/ActionExecutionInfo`` also carries the field, allowing a
monitor subscribed to ``action_execution_info`` to observe which branch a
policy took without subscribing to the actions hub.

An observation originating elsewhere, such as a perception node operating
independently of the acting robot, is supplied by binding ``observed`` in a
deployment-specific action template. Both routes are covered by the
integration tests; see :doc:`../reports/epistemic_end_to_end`.

Epistemic nodes
---------------

Four node types are registered by the plugin library
``libplansys2_epistemic_bt_nodes.so``.

``CheckKnowledge``
   A condition node, the counterpart of ``CheckOverAllReq`` for conditions the
   problem expert cannot answer. "The corridor is clear" is a fact and lives
   there; "r1 knows whether the corridor is clear" is not a fact about the
   corridor, and no set of predicates records it. Ports: ``node``, the index of
   the policy node being guarded, and ``action``, the action identifier used in
   the error message.

``ApplyEpistemicUpdate``
   The counterpart of ``ApplyAtEndEffect``, one level up. An action changes
   what agents know by more than its own effects: an announcement informs
   everyone listening, and an observation that finds nothing still rules worlds
   out. This performs the DEL product update, so the next action's guard is
   checked against the state this action produced. Input ports: ``node``,
   ``action``, and ``observed``, the outcome the performer observed, which the
   packaged action template binds to ``ExecuteAction``'s outcome port and which
   is left empty to let the state decide. Output port: ``outcome``, the outcome
   that occurred, which the following ``EpistemicSwitch`` reads. The two are
   separate blackboard entries: ``observed`` holds the performer's report and
   ``outcome`` holds the state's reading of it.

``EpistemicSwitch``
   A control node with no PlanSys2 counterpart. It reads the outcome the update
   reported and runs the continuation planned for it. Ports: ``node``,
   ``outcome``, the observed outcome, and ``outcomes``, the outcomes in child
   order, separated by semicolons. An outcome the policy does not list fails
   the node rather than defaulting to a branch: every branch was built for a
   different belief, and running one anyway is a robot acting confidently on a
   belief nothing supports. The failure reaches the executor, whose answer to a
   failed plan is to replan.

``CheckEpistemicGoal``
   A condition node that runs once, after the policy, with a single ``goal``
   port, empty for none. A tree with no epistemic goal, which is to say a
   classical plan, succeeds here, since the executor already checks the PDDL
   goal. Reaching a leaf means one execution finished, not that the goal holds:
   every leaf was believed to reach it, but that belief was formed against the
   model at planning time.

Epistemic state
---------------

``epistemic_state_node`` runs a lifecycle node named ``epistemic_state``. It is
to knowledge what the problem expert is to facts: it holds one pointed Kripke
model at a time, for one grounded task, and answers six services. Each of them
is reachable from a terminal through ``ros2 plansys2 epistemic``, described in
:doc:`../getting_started/command_line`.

.. list-table::
   :header-rows: 1
   :widths: 38 62

   * - Service
     - Question
   * - ``epistemic_state/load_task``
     - Be the model of this task. The task is supplied inline as ``task_json``,
       by path as ``task_file``, or as the EPDDL sources to ground
       (``epddl_domain``, ``epddl_problem``, ``epddl_libraries``); loading
       resets the model to that task's initial state. The response reports the
       number of worlds, agents and actions loaded.
   * - ``epistemic_state/check_formula``
     - Does this formula hold now? The response distinguishes a failure, no
       task loaded or a formula that does not parse, from the answer itself.
   * - ``epistemic_state/apply_action``
     - This action ran: perform the DEL product update and report what was
       observed. The request names the action in the task's own vocabulary and
       may carry the observed outcome; the response reports the outcome that
       occurred and the size of the resulting state.
   * - ``epistemic_state/get_goal``
     - What is being aimed at, and whether it holds yet. The response says
       whether the goal is the loaded task's own or one set since.
   * - ``epistemic_state/set_goal``
     - Aim at something else, without re-grounding the problem. The formula is
       parsed against the loaded task, so a goal naming an agent or atom the
       task does not have is refused here rather than failing later inside a
       planning request. An empty goal restores the task's own.
   * - ``epistemic_state/announce``
     - Everyone just learned that this is true. The model is restricted to the
       worlds where the formula holds. An announcement that holds nowhere the
       state considers possible is refused rather than emptying the model.

It declares the same four EPDDL parameters as the solver, plus ``task_file``,
and publishes on ``epistemic_state/state`` with transient-local durability. A
task named at configure time is the common case: one mission, one task, loaded
before anything asks a question about it. Pointing this node and the planner at
the same pair of ``.epddl`` files is what keeps the policy and the model it is
checked against expressed in one vocabulary. The failure the arrangement is
there to prevent is a policy naming an action the state has never heard of.

It is a managed node of the system rather than something started alongside it.
Both launch files start it on ``epistemic_state:=True``, and the lifecycle
manager configures and activates it with the other four and takes it down with
them. It is off by default, since a classical system has no use for it.

It runs as its own process even under the monolithic launch, where the other
four share one. That is not an oversight: ``plansys2_node`` manages it through
``LifecycleServiceClient``, which needs the node's name and not its class, so
``plansys2_bringup`` links against nothing epistemic and the package every
other distribution's workflow builds is unchanged. The same reasoning as
``bt_node_plugins`` on the executor, one level up.

The switch is a single launch argument rather than a parameter because it
decides two things that must agree: whether the process is started, and
whether the manager waits for it. A parameters file can set the second but not
the first, so the argument overrides the parameters file rather than the other
way round.

``startup_function`` in ``plansys2_lifecycle_manager`` brings up whatever map
it is given, skipping a name it does not find, which is what lets one function
serve a four-node classical bringup, a five-node epistemic one, and the
six-node system that adds perception. The standalone
``lifecycle_manager_node`` the distributed launch runs takes the set as its
``managed_nodes`` parameter for the same reason; the distributed launch
assembles that set in Python, once the launch arguments have values, because
naming a node that was not started makes startup block and then fail.

Announcing is the counterpart of the problem expert's ``set predicate``, and it
is deliberately not the same operation. Setting a predicate changes what is
true; announcing changes what is *known*. Every agent that could not previously
tell the surviving worlds apart from the ruled-out ones now can, and knows that
the others can too, which is why announcing ``muddy_c1`` in the muddy-children
model makes ``(K c1 muddy_c1)`` hold when it did not before. Knowledge that
reaches only one agent is not a public announcement and cannot be expressed this
way; that needs an event model, which is what an action is.

The goal it holds is what the plan solver plans for. The solver subscribes to
``epistemic_state/state``, where the goal travels latched, and replaces the
task's goal with it when the two differ. It listens rather than asks because
planning runs inside the planner's own service callback, and a service call
from there can deadlock a single-threaded executor. Setting
``goal_from_state`` to false plans for the problem exactly as written.

The state advances by executed actions rather than by watching the world, which
is what makes it a belief state rather than a log. When it disagrees with what
the robot observed, the disagreement surfaces at ``apply_action`` as an outcome
the model cannot account for: a reason to replan, not to overwrite the model
quietly.

Which outcome occurred is a question about the world, not about the model. When
the state designates a single world it already answers it, and the response
reports it. When it designates several it genuinely does not know, and the
observation has to come from whoever did the sensing.

Perception
----------

The state advances by executed actions, and something still has to tell it what
a sensing action saw. ``plansys2_epistemic_perception`` is that something: a
lifecycle node that watches an ``nav_msgs/OccupancyGrid``, classifies named
regions of it, and reports what it found in the vocabulary of the loaded task.

Two things stand between a grid and a formula, and the package exists because
neither is mechanical.

The first is a quantifier. A cell is free or occupied; a proposition is about a
region, and which quantifier joins them is a modelling decision:

.. list-table::
   :header-rows: 1
   :widths: 20 80

   * - The region is
     - when
   * - ``Clear``
     - every cell in it has been observed and settled as free
   * - ``Blocked``
     - any one cell in it is occupied
   * - ``Unknown``
     - anything else

Occupied beats unobserved, since no further looking removes an obstacle;
unobserved beats free, since a region is clear only when all of it is. A cell
that has been seen without being settled, the band between the two
thresholds, counts as unobserved, because calling it free would hand an agent knowledge
it does not have.

The second is the vocabulary. A region is a place on a map; an atom is a name
in a grounded task, and those names are flat: ``blocked``, ``at-junction_r1``.
Each region therefore names the atom its occupancy decides and says which way
that atom points, which is what lets the corridor of the fleet tutorial be
``blocked`` rather than ``clear_corridor``:

.. code-block:: yaml

   epistemic_perception:
     ros__parameters:
       regions: ["corridor"]
       corridor:
         boxes: [2.0, -0.5, 8.0, 0.5, 7.5, 0.5, 8.5, 4.0]
         atom: "blocked"
         atom_true_when_clear: false
         sensing_action: "inspect_r1"
         outcome_when_clear: "e-inspect-clear"
         outcome_when_blocked: "e-inspect-blocked"

Regions are given in metres in the map frame rather than in cells because a map
is re-gridded as it grows: the origin moves and the width changes, and a region
written in cell indices would come to mean somewhere else without saying so.

How the finding is reported follows from why it is being reported, which is the
same distinction announcing draws above. A region bound to a sensing action
reports through ``apply_action``: the observation belongs to an action the
planner branched on, the state has its event model, and the update is the one
the plan accounted for. A region without a binding is announced instead, as
information that arrived outside the plan: an operator or two robots
reconciling their maps, which is exactly what a public announcement is for.

It reports on change rather than on every grid. A map arrives several times a
second and says the same thing each time; a repeated announcement is harmless
in the model, since restricting to worlds where a formula already holds changes
nothing, but a sensing action applied twice is not, and both routes follow the
one rule. A region that goes back to undecided produces no call at all: the
state has no operation for taking knowledge back.

It is brought up the way the state is. Both launch files start it on
``epistemic_perception:=True``, as a sixth managed node, and the lifecycle
manager configures and activates it with the others. It is off by default, and
it starts watching nothing until the parameters file names regions under
``epistemic_perception:``.

Asking for it without ``epistemic_state:=True`` is refused rather than
started. Every route perception has for reporting is a call on the state, so
the pair is not a degraded system but one that cannot work at all, and both the
distributed launch and ``plansys2_node`` say so instead of bringing up a node
with nowhere to report. As with the state, ``plansys2_bringup`` manages it by
name over services and links against none of it.

Regions are declared at configure time rather than at construction, since there
is no list of regions before the parameters are read. Configure is reachable
more than once, so the node declares each region's settings only if they are
not already declared, and ``on_cleanup`` releases the subscription and the
service client that the configuration created. A box is accepted as an integer
array as well as a double array, because ``[2, -1, 8, 1]`` in a YAML file is a
list of integers and a reader writing metres has no reason to expect otherwise.

Executor configuration
----------------------

Two parameters connect the executor to all of this:

.. code-block:: yaml

   executor:
     ros__parameters:
       bt_builder_plugin: "EpistemicBTBuilder"
       bt_node_plugins: ["libplansys2_epistemic_bt_nodes.so"]

``bt_node_plugins`` is a generic hook: the executor loads BehaviorTree.CPP node
libraries before building a tree. That is what keeps ``plansys2_executor`` free
of any dependency on the epistemic stack. It never links against the epistemic
nodes, it loads them, and the nodes find the policy on the blackboard where the
executor already publishes the plan it is running.

Package boundaries
------------------

.. list-table::
   :header-rows: 1
   :widths: 34 46 20

   * - Package
     - Contents
     - Depends on ``plansys2_executor``
   * - ``plansys2_epddl_grounder``
     - The EPDDL front end: runs plank, caches the result
     - No
   * - ``plansys2_epistemic_msgs``
     - The three service definitions
     - No
   * - ``plansys2_epistemic_planner``
     - Kripke states, product update, contraction, heuristics, search, the
       solver plugin
     - No
   * - ``plansys2_epistemic_executor``
     - Policy, tree rendering, the four nodes, the state node
     - No
   * - ``plansys2_epistemic_perception``
     - Regions of a map, and their translation into calls on the state
     - No
   * - ``plansys2_epistemic_bt_builder``
     - The ``BTBuilder`` plugin
     - Yes
   * - ``plansys2_aletheia_plan_solver``
     - The subprocess plan solver plugin
     - No
   * - ``plansys2_tui_cli``
     - The ``ros2 plansys2`` verbs and the Textual dashboard
     - No
   * - ``eplansys``
     - Metapackage; dependencies only, no code
     - No

The builder is a separate package because it is the only piece that needs the
executor's plugin API. Keeping it apart lets everything else build and be
tested against a released distribution rather than only against this fork.

Within ``plansys2_epistemic_executor`` the same boundary is drawn once more.
The policy and its rendering as a behavior tree are built as a separate
library, ``plansys2_epistemic_executor_policy``, which depends on nothing but
the message package: they are the shared reading of a policy, and keeping them
free of ROS lets them be tested as the pure transformation they are.
