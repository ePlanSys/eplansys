Architecture
============

ePlanSys attaches to PlanSys2 :cite:p:`martin2021plansys2` only through its
extension points (:numref:`fig-architecture`):

* the planner is a plan solver plugin (:ref:`arch-solver`);
* the tree builder is a behavior tree builder plugin (:ref:`arch-bt`);
* the epistemic behavior tree nodes are a BehaviorTree.CPP :cite:p:`btcpp`
  plugin library (:ref:`arch-nodes`);
* the model of what the agents know is a lifecycle node,
  ``epistemic_state`` (:ref:`arch-state`);
* perception is a second lifecycle node that reports to the first
  (:ref:`arch-perception`).

``plansys2_executor`` links against none of these. Every attachment is made
through pluginlib or a parameter, and a classical deployment loads none of
them.

Three upstream interfaces were extended to carry a policy through the system:
``PlanItem`` has fields for branching, ``ActionExecution`` has a field for the
outcome a performer observed, and the executor loads behavior tree node
libraries named by parameter. All three are additions whose defaults reproduce
the classical behaviour. ``NOTICE`` lists these and the remaining differences
from upstream.

.. _fig-architecture:

.. graphviz::
   :caption: Attachment points of the epistemic packages in PlanSys2.

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

.. _arch-solver:

Plan solver plugin
------------------

``plansys2/EpistemicPlanSolver`` implements ``plansys2::PlanSolverBase`` and is
exported by the plugin description in ``plansys2_epistemic_planner``. It is
selected in the planner's parameters:

.. code-block:: yaml

   planner:
     ros__parameters:
       plan_solver_plugins: ["EPISTEMIC"]
       EPISTEMIC:
         plugin: "plansys2/EpistemicPlanSolver"

Parameters are declared under the plugin name at configure time
(:numref:`tab-solver-params`).

.. _tab-solver-params:

.. list-table:: Parameters of ``EpistemicPlanSolver``.
   :header-rows: 1
   :widths: 22 12 14 52

   * - Parameter
     - Type
     - Default
     - Meaning
   * - ``epddl_domain``
     - string
     - empty
     - Path to the EPDDL domain.
   * - ``epddl_problem``
     - string
     - empty
     - Path to the EPDDL problem.
   * - ``epddl_libraries``
     - string[]
     - empty
     - Action-type libraries the domain declares. Empty selects the
       ``intermediate`` library packaged with ``plansys2_epddl_grounder``.
   * - ``plank_command``
     - string
     - empty
     - Path to the plank binary. Empty: ``$PLANK``, then ``PATH``.
   * - ``task_file``
     - string
     - empty
     - Absolute path to a grounded task (JSON). Ignored when the EPDDL sources
       are set.
   * - ``heuristic``
     - string
     - empty
     - One of ``ug``, ``ed``, ``ks``, ``wc``, ``rpg``, ``radd``, ``kadd``,
       ``kff``. Empty: chosen by the selection policy.
   * - ``strategy``
     - string
     - empty
     - One of ``gbfs``, ``ehc``, ``aostar``, ``replan``, ``portfolio``. Empty:
       chosen by the selection policy; AO* selected this way on a task with
       sensing actions runs for a short pass before ``replan`` takes the rest
       of the timeout.
   * - ``policy_file``
     - string
     - empty
     - JSON selection policy replacing the built-in rule table
       (:doc:`epistemic_models`).
   * - ``action_mapping``
     - string
     - empty
     - JSON map from grounded action names to PlanSys2 action expressions.
       Empty: naming convention (:doc:`../getting_started/first_plan`).
   * - ``conditional_plan``
     - string
     - ``flatten``
     - Treatment of a branching solution: ``policy``, ``flatten`` or
       ``reject``. The parameters file in ``plansys2_bringup`` sets
       ``policy``.
   * - ``temporal``
     - bool
     - ``false``
     - Plan with the action mapping's durations, minimising the worst-case
       makespan; overlap only actions whose updates commute
       (:ref:`arch-temporal`). Requires an executor with the epistemic BT
       builder of 0.4.0 or later.
   * - ``max_concurrency``
     - int
     - ``3``
     - Largest number of actions running at once under ``temporal``. The
       commutation check enumerates subsets of the running set.
   * - ``temporal_estimate``
     - string
     - ``admissible``
     - ``admissible``: the least worst-case makespan. ``critical_path``:
       deepen with :math:`\max(h_{\mathrm{adm}}, h_{\mathrm{cp}})`, which
       weighs each layer of the relaxed closure by its shortest contributing
       action; the policy is admissible but not necessarily the shortest.
   * - ``temporal_bound_growth``
     - double
     - ``1.0``
     - Factor by which the search's time bound grows between iterations.
       Optimality does not depend on it; ``1.0`` steps to the next lower
       bound.
   * - ``consistent_beliefs``
     - bool
     - ``true``
     - Reject states in which an agent believes a contradiction (empty
       accessibility set). Such an agent satisfies every belief goal
       vacuously. ``false`` accepts these plans, as plank does.
   * - ``goal_from_state``
     - bool
     - ``true``
     - Plan for the goal published by ``epistemic_state``
       (:ref:`arch-state`).
   * - ``initial_from_state``
     - bool
     - ``true``
     - Plan from the model published by ``epistemic_state``
       (:ref:`arch-replan-belief`).

Every solution is validated against the task before it is returned; a plan
that fails validation is not returned.

The planner node answers planning requests asynchronously: the service
callback passes the request to a dedicated planning thread and returns, and
requests are searched one at a time. In the monolithic bring-up, where one
executor serves every PlanSys2 node and the lifecycle manager, a search inside
the callback would block bring-up for the duration of the search.

Subprocess variant
~~~~~~~~~~~~~~~~~~

``plansys2_aletheia_plan_solver`` provides ``plansys2/AletheiaPlanSolver``,
which runs the planner binary as a subprocess, in the manner of
``plansys2_popf_plan_solver``: it writes the task to a file, runs the process
under the solver timeout, and reads the plan back.

.. code-block:: yaml

   planner:
     ros__parameters:
       plan_solver_plugins: ["ALETHEIA"]
       ALETHEIA:
         plugin: "plansys2/AletheiaPlanSolver"
         command: "/abs/path/to/epistemic_planner"
         conditional_plan: "policy"

It accepts the parameters of :numref:`tab-solver-params` with the same
meanings, plus ``command`` (default ``epistemic_planner`` on ``PATH``),
``arguments`` (appended verbatim) and ``output_dir`` (task, plan and log). A
parameters file moves between the two plugins by changing ``plugin``.

The plan file contains action names and event indices only. Branch outcomes,
knowledge requirements and the goal are properties of the task, so the plugin
parses the task itself and serialises the policy with
``plansys2_epistemic_planner``'s code. It validates the returned plan against
its own parse of the task; a disagreement indicates that the binary and the
workspace were built from different planner revisions. The ``aletheia``
package installs both from one checkout, and a test holds the two plugins to
the same policy on the fleet tasks.

The in-process plugin is the default. The subprocess plugin is intended for a
planner versioned separately from the workspace, run under its own resource
limits, or compared against the in-process build.

.. _arch-policy:

Policies
--------

A policy is a tree transported in the epistemic fields of
``plansys2_msgs/PlanItem`` (:numref:`tab-planitem`).

.. _tab-planitem:

.. list-table:: Epistemic fields of ``plansys2_msgs/PlanItem``.
   :header-rows: 1
   :widths: 30 70

   * - Field
     - Contents
   * - ``children``
     - Item index of the continuation for each outcome.
   * - ``outcomes``
     - Event name selecting each continuation, in the same order.
   * - ``sensing``
     - Whether the outcome must be observed.
   * - ``knowledge_requirements``
     - Epistemic formulas the action requires to hold.
   * - ``epistemic_action``
     - Action name in the planner's vocabulary.
   * - ``starts_after``, ``given_items``, ``given_outcomes``
     - Temporal policies only (``Plan.temporal``): the items an item waits
       for, and the observations selecting its branch
       (:numref:`tab-temporal-fields`).

A classical plan leaves these fields empty and is read as PlanSys2 writes it:
item :math:`i` followed by item :math:`i+1`. Every plan is thus a policy; a
classical plan is one without branches.

.. _arch-bt:

Behavior tree
-------------

``plansys2::EpistemicBTBuilder`` renders each policy node as the unmodified
PlanSys2 action subtree, extended with three elements a sequence cannot
express: a guard on required knowledge, the epistemic update the action
performs, and a branch on the observed outcome (:numref:`lst-bt`).

.. _lst-bt:

.. code-block:: text
   :caption: Rendering of a sensing action with two outcomes.

   Sequence "node_0 (peek A)"
     CheckKnowledge         node="0"
     WaitAtStartReq         action="(peek A):0"           ┐
     ApplyAtStartEffect     action="(peek A):0"           │
     ReactiveSequence                                     │ PlanSys2
       CheckOverAllReq      action="(peek A):0"           │ action subtree
       ExecuteAction        action="(peek A):0"           │
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

A node with one continuation is rendered without a switch, so a classical plan
yields the flat sequence that PlanSys2's ``SimpleBTBuilder`` produces.

``get_graph`` reports the policy as a temporal graph with one node per policy
node and one arc per continuation. ``SimpleBTBuilder`` derives the ordering of
a sequential plan from causal links; a policy states its ordering explicitly,
so the builder records it, and ``propagate`` computes the bounds from the
durations the plan carries.

The lower bound of every arc is the full duration of the parent action.
``CheckKnowledge`` of a continuation is evaluated against the state produced by
the parent's ``ApplyEpistemicUpdate``; overlap would evaluate the guard against
a state that does not yet exist. An action therefore never overlaps its own
continuation.

Runs of the policy that are mutually unordered may overlap. The builder
dispatches them in a ``Parallel`` group (``schedule_policy``) and hoists an
independent action above a branch point when every branch begins with it. Two
actions may share a group when their PDDL effects do not interfere and the test
in ``policy_parallel.cpp`` passes: they act for disjoint sets of agents, and
neither action's knowledge requirements mention the other's agents.

.. note::

   The independence test is syntactic and does not establish that the two
   epistemic updates commute. A group applies its updates in completion
   order; an announcement that writes no atom can still disable another
   agent's sensing action whose precondition is that the agent does not yet
   know.

.. _arch-temporal:

Temporal policies
-----------------

With ``temporal`` set, the solver plans with durations. The durations are the
action mapping's, which the executor also schedules and times out against. A
durative action is checked for applicability when it starts and updates the
model when it ends; a sensing action's outcome is known at its end, and the
policy branches there.

Two actions *commute* in a state when, for every pair of their outcomes, each
remains applicable after the other, the same outcome pairs are reachable in
both orders, and the two resulting models are bisimilar. The solver decides
this exactly in each state: it computes both orders, contracts each result and
compares canonical fingerprints. A running set is *coherent* when, in every
state it can pass through as its actions end, every remaining pair commutes.
The solver starts an action only if the running set remains coherent. Under
coherence, the model after a set of overlapping actions does not depend on the
order in which they end.

The search minimises the worst-case makespan over outcomes. Its result can
differ from the untimed policy: on the corridor, both robots inspect at once
(35 s), where the shortest untimed policy, inspect and announce, takes 36 s.

The syntactic test of the previous section admits pairs that do not commute.
On the corridor, after r2 has observed the corridor blocked, ``report-blocked_r2``
and ``inspect_r1`` pass it; the announcement makes r1 know, and ``inspect_r1``
requires that r1 does not know. Over the reachable states of the fleet depot
to depth 4, the syntactic test admits 1476 pairs, of which 72 do not commute.

A temporal policy is a ``plansys2_msgs/Plan`` with ``temporal`` set. Each item
carries the fields of :numref:`tab-temporal-fields`; ``children`` and
``outcomes`` are empty.

.. _tab-temporal-fields:

.. list-table:: Fields of a temporal policy item.
   :header-rows: 1
   :widths: 25 75

   * - Field
     - Meaning
   * - ``time``
     - Planned start, for display and scheduling estimates.
   * - ``starts_after``
     - Items the item waits for: every item on its path planned to end before
       it starts, less those another listed item already waits for. Empty:
       the item starts with the policy.
   * - ``given_items``, ``given_outcomes``
     - The branch: the item runs only if each listed item observed the
       outcome at the same position. Every observation on the path is listed.

The builder renders every item under one ``Parallel``, each gated as in
:numref:`code-temporal-item`.

.. _code-temporal-item:

.. code-block:: xml
   :caption: Rendering of one temporal policy item.

   <Sequence name="item_4">
     <WaitForEnds items="0;1"/>
     <IfThenElse>
       <CheckGiven after="0;1" items="0" outcomes="e-inspect-clear"/>
       <Sequence>  <!-- action block -->  <MarkDone node="4" status="ran"/></Sequence>
       <MarkDone node="4" status="skipped"/>
     </IfThenElse>
   </Sequence>

Dispatch is event-driven: an item starts when the items it waits for have
ended, whatever their actual durations. Dispatch at planned start times can
start an action while an action planned to end earlier is still running, an
overlap the solver did not check. An item whose branch was not observed is
marked skipped; items waiting for it are then skipped in turn.

.. _arch-observation:

Reporting an observation
------------------------

The epistemic state determines the outcome of a sensing action by itself only
when its model designates a single world. Otherwise the outcome is supplied by
the agent that performed the sensing.

The observation is carried by the existing execution protocol.
``plansys2_msgs/ActionExecution`` has an ``outcome`` field, meaningful with
``FINISH``, which the performer sets on completion:

.. code-block:: cpp

   // in the performer's do_work, once the sensor has answered
   finish(true, 1.0, "Corridor inspected", clear ? "e-inspect-clear"
                                                 : "e-inspect-blocked");

The token must name an event of the action's event model. ``status`` remains a
human-readable message. Ordinary actions leave ``outcome`` empty, which is the
default, so classical performers need no change.

``ExecuteAction`` exposes the value on an ``outcome`` output port; the
packaged action template binds it to the ``observed`` input of
``ApplyEpistemicUpdate``, with one blackboard entry per policy node so that
sensing actions on different branches do not overwrite each other.
``plansys2_msgs/ActionExecutionInfo`` carries the field as well, so a monitor
of ``action_execution_info`` can observe which branch a policy took.

An observation from another source, such as a perception node independent of
the acting robot, is supplied by binding ``observed`` in a deployment-specific
action template. Both routes are covered by the integration tests
(:doc:`../reports/epistemic_end_to_end`).

.. _arch-nodes:

Epistemic behavior tree nodes
-----------------------------

The plugin library ``libplansys2_epistemic_bt_nodes.so`` registers five node
types (:numref:`tab-bt-nodes`).

.. _tab-bt-nodes:

.. list-table:: Epistemic behavior tree nodes.
   :header-rows: 1
   :widths: 24 14 22 40

   * - Node
     - Kind
     - Ports
     - Semantics
   * - ``CheckKnowledge``
     - condition
     - ``node``, ``action``
     - Succeeds iff every knowledge requirement of policy node ``node`` holds
       in the current state. The epistemic counterpart of
       ``CheckOverAllReq``: :math:`\mathit{Kw}_{r_1}\,\mathit{blocked}` is
       not a fact about the corridor and has no PDDL representation.
   * - ``ApplyEpistemicUpdate``
     - action
     - in: ``node``, ``action``, ``observed``, ``recover``;
       out: ``outcome``
     - Applies the product update for the action with the observed outcome
       (empty: the state determines it) and writes the outcome that occurred.
       ``observed`` is the performer's report; ``outcome`` is the state's
       reading of it. ``recover`` (default true) requests model repair
       (:ref:`arch-contradiction`).
   * - ``EpistemicSwitch``
     - control
     - ``node``, ``outcome``, ``outcomes``
     - Runs the child whose position in ``outcomes`` (``;``-separated)
       matches ``outcome``. An unlisted outcome fails the node; no default
       branch is taken, since every branch was planned for a different belief.
       The executor responds to the failure by replanning.
   * - ``CheckBeliefUnchanged``
     - condition
     - ``action``, ``enabled``
     - Fails when the belief has been changed by an announcement since the
       policy started (:ref:`arch-announcements`). ``enabled=false``
       disables the check.
   * - ``CheckEpistemicGoal``
     - condition
     - ``goal``
     - Runs once after the policy. Succeeds iff ``goal`` holds; an empty goal
       (classical plan) succeeds, the PDDL goal being checked by the
       executor. Reaching a leaf only means the execution finished, since the
       leaf was planned against the model at planning time.

.. _arch-domain:

Domain introspection
--------------------

In EPDDL an action is an event model with per-agent observability, which has
no PDDL representation; the classical ``get domain action details`` cannot
report it. Two services on the ``epistemic_state`` node do. They reside on the
node that holds the task because an EPDDL domain and problem are grounded
together into one artefact.

``epistemic_domain/get_domain``
   Agents, atoms, grounded actions, and which actions are sensing (have more
   than one event that can occur). Also the frame (S5 or KD45) and whether any
   action gives two agents different observability.

``epistemic_domain/get_action_details``
   One action's event model: each event with its precondition and effects,
   which events can occur, and each agent's observability. For
   ``inspect_r1`` in the corridor mission::

      r1: sees which event occurred
      r2: sees nothing of which event occurred

The terminal exposes both as ``ros2 plansys2 epistemic domain`` and
``ros2 plansys2 epistemic action <name>``.

.. _arch-state:

Epistemic state
---------------

``epistemic_state_node`` runs the lifecycle node ``epistemic_state``. It holds
one multi-pointed Kripke model for one grounded task, in the role the problem
expert plays for facts, and provides the services of
:numref:`tab-state-services` in addition to those of :ref:`arch-domain`. All are
reachable through ``ros2 plansys2 epistemic``
(:doc:`../getting_started/command_line`).

.. _tab-state-services:

.. list-table:: Services of ``epistemic_state``.
   :header-rows: 1
   :widths: 38 62

   * - Service
     - Semantics
   * - ``epistemic_state/load_task``
     - Load a task, given inline (``task_json``), by path (``task_file``), or
       as EPDDL sources (``epddl_domain``, ``epddl_problem``,
       ``epddl_libraries``), and reset the model to its initial state.
       Returns the number of worlds, agents and actions.
   * - ``epistemic_state/check_formula``
     - Evaluate a formula in the current model, optionally from one agent's
       perspective (``agent``). A failure (no task, parse error) is reported
       separately from a false result.
   * - ``epistemic_state/apply_action``
     - Apply the product update for a named action, optionally with the
       observed outcome and ``allow_recovery``. Returns the outcome that
       occurred, the size of the resulting model, and ``recovered``.
   * - ``epistemic_state/get_goal``
     - The current goal, whether it holds, and whether it is the task's own
       or one set since.
   * - ``epistemic_state/set_goal``
     - Replace the goal without re-grounding. The formula is parsed against
       the task, so unknown agents or atoms are rejected here. An empty goal
       restores the task's goal.
   * - ``epistemic_state/announce``
     - Public announcement: restrict the model to the worlds where the formula
       holds. Rejected if no designated world satisfies it.
   * - ``epistemic_state/get_agent_perspective``
     - The model as seen by one agent (:ref:`arch-perspective`).

The node declares the solver's four EPDDL parameters and ``task_file``, and
publishes ``epistemic_state/state`` with transient-local durability. The
message carries the goal, ``belief_version`` and, unless ``publish_model`` is
false, the model. The node and the solver are configured with the same EPDDL
files so that the policy and the model it is checked against share one
vocabulary; each grounds the sources itself, which removes any dependency on
start-up order.

Lifecycle
~~~~~~~~~

Both bring-up launch files start the node when ``epistemic_state:=True``
(default false), and the lifecycle manager configures, activates and shuts it
down with the four PlanSys2 nodes. It runs in its own process even under the
monolithic launch: ``plansys2_node`` manages it by name through
``LifecycleServiceClient``, so ``plansys2_bringup`` has no build dependency on
the epistemic packages. The switch is a launch argument because it controls both whether the process starts and whether the manager
waits for it.

``startup_function`` in ``plansys2_lifecycle_manager`` brings up the nodes it
is given and skips names it does not find, so one function serves the
four-node classical, five-node epistemic, and six-node (with perception)
configurations. The standalone ``lifecycle_manager_node`` used by the
distributed launch takes the set as ``managed_nodes``; the launch assembles it
after argument resolution, since naming a node that was not started blocks
start-up.

Announcement and goal
~~~~~~~~~~~~~~~~~~~~~

``announce`` is the epistemic counterpart of the problem expert's
``set predicate`` and is a different operation: setting a predicate changes
what is true, announcing changes what is known. After a public announcement of
:math:`\varphi` every agent knows :math:`\varphi` and knows that the others
know it; announcing ``muddy_c1`` in the muddy-children model makes
:math:`K_{c_1}\,\mathit{muddy}_{c_1}` hold. Information that reaches only some
agents is not a public announcement and requires an event model, i.e. an
action.

The solver subscribes to ``epistemic_state/state`` and replaces the task's goal
with the published one when they differ. The goal is received by subscription,
so the search never waits on another node. ``goal_from_state:
false`` plans for the problem as written.

.. _arch-announcements:

Information from outside the plan
---------------------------------

A policy is computed against a belief, and its own actions change that belief
in ways it has branches for. An announcement (perception resolving a region, an
operator, two robots reconciling maps) changes it in a way the policy did not
account for, and a behavior tree has no means to observe a service call.

The state therefore counts the announcements it has applied and publishes the
count as ``belief_version``; ``CheckBeliefUnchanged`` compares it with the
value at policy start. The packaged action template places the node inside the
``ReactiveSequence``, so it is re-evaluated while the action runs and the
policy fails when the information arrives. The executor then replans from the
announced model.

``apply_action`` does not increment the version: a policy's own actions are
anticipated by its branches. A deployment that prefers to finish the current
policy sets ``enabled`` to false.

.. _arch-replan-guard:

Replanning without progress
---------------------------

A plan that fails, is replanned, and fails identically forms a loop.
Epistemic missions are prone to it: an observation the model cannot represent
fails the update, and a domain that cannot express the outcome will not plan
for it on the next attempt.

The executor counts replans that follow no completed action. Replans in
themselves are not the criterion; a mission that replans while making progress
is healthy. When the count exceeds ``max_replans_without_progress`` (default
3) the plan is aborted through the ordinary failure path. ``0`` disables the
bound. The synthetic ``(INIT)`` entry of every action map is not counted as
progress.

Model repair (:ref:`arch-contradiction`) and this guard are complementary:
repair allows the next plan to differ; the guard stops the case where it does
not.

.. _arch-supervisor:

Mission supervision
-------------------

``plansys2::MissionSupervisor`` in ``plansys2_executor`` implements the outer
loop: plan, execute, and on failure plan again from the problem and the belief
the failed attempt left.

.. code-block:: cpp

   plansys2::MissionSupervisor supervisor(node);   // reads mission.* parameters
   supervisor.wait_for_system();
   supervisor.problem()->setGoal(plansys2::Goal("(and(reported r1))"));
   const auto report = supervisor.run();           // Succeeded, GaveUp, NoPlan, ...

.. list-table:: Parameters of ``MissionSupervisor``.
   :header-rows: 1
   :widths: 40 12 48

   * - Parameter
     - Default
     - Meaning
   * - ``mission.max_attempts_without_progress``
     - 3
     - Consecutive attempts in which no action completed.
   * - ``mission.max_attempts``
     - 0
     - Total attempts; 0 for no bound.
   * - ``mission.settle_seconds``
     - 2
     - Delay before the next planning request.
   * - ``mission.check_rate_hz``
     - 4
     - Rate at which execution is polled.

The retry decision (``MissionRetryPolicy``) uses the same criterion as the
executor's guard and is tested without a running system. An empty plan from a
replan means the goal already holds and ends the mission successfully. All
three demo missions use the supervisor; ``plansys2_tests/test_7`` runs it
against a stale map, where the first policy is stopped by its knowledge guard
and the second, planned from the repaired belief, reports the corridor
blocked.

.. _arch-contradiction:

Observations that contradict the model
--------------------------------------

The model is a belief and can be wrong: the map says a corridor is clear, the
robot inspects it and observes that it is blocked, and ``apply_action``
receives an outcome the model holds impossible. Rejecting the update leaves
the model unchanged, and the following replan would start from the belief the
sensor has just refuted.

With ``allow_recovery`` the state repairs the model instead, giving precedence
to the observation: the model is re-designated to the worlds in which the
observed event is applicable, and the update proceeds. The response sets
``recovered`` and describes what was discarded; the state logs the repair at
warning level. ``ApplyEpistemicUpdate`` requests recovery by default; its
``recover`` port disables it.

Re-designation changes which worlds are considered actual; it cannot add a
world the model does not contain. An observation that no world could produce
is rejected regardless of ``allow_recovery``. In the corridor domain,
inspecting before driving to the junction is such a case: every world has the
robot away from the junction.

.. _arch-replan-belief:

Replanning from the current belief
----------------------------------

``epistemic_state/state`` also carries the model, under ``model``, in the shape
of a task's initial state. With ``initial_from_state`` (default true) the
solver plans from it, so that a replan starts from the belief the mission has
reached.

An epistemic policy typically fails because the world contradicted its belief.
With recovery enabled the update succeeds and repairs the model, and the
failure occurs at the next action's ``CheckKnowledge``, the action having been
planned for a belief the repaired model no longer supports.
``EpistemicSwitch`` also rejects unplanned outcomes, but only branching nodes
have a switch, and a node that branches on every outcome has none unplanned.
Planning from the grounded initial model would restart from a belief already
refuted, sending the robot to re-sense what it has observed or to act on a
possibility it has eliminated.

The published model is resolved against the task before use. Every atom, agent
and designated world it names must exist in the task; an unknown name is an
error, since it indicates that the state and the planner hold different
problems. A model with no designated world is rejected: every formula holds
vacuously in it.

``initial_from_state: false`` on the solver, or ``publish_model: false`` on the
state, disables the mechanism; in the second case the solver falls back to the
task's initial state.

The state advances only by executed actions. A
disagreement with what the robot observed therefore appears at
``apply_action`` as an outcome the model cannot account for, which is a reason
to repair or replan.

.. _arch-perspective:

Agent perspective
-----------------

The state holds one model for the whole system, in which nested
group knowledge (":math:`r_1` knows that :math:`r_2` does not know") is
expressible. A deployment may also need the model an individual agent acts on.

``epistemic_state/get_agent_perspective`` returns the same worlds and relations
with the designated set replaced by the worlds the agent considers possible
from the actual worlds, in the task's initial-state format, so it can be given
to a planner as a replan's initial model. ``check_formula`` evaluates from the
same perspective when ``agent`` is set: from the system's perspective
``blocked`` is a statement about the corridor; from ``r1``'s, it holds only if
``r1`` believes it.

The response also states whether the agent considers the actual world
possible. Under S5 this always holds: an agent may be uninformed but not
mistaken. Under KD45 it need not. The terminal exposes this as ``ros2 plansys2
epistemic view <agent>`` and ``check --agent``.

.. _arch-perception:

Perception
----------

``plansys2_epistemic_perception`` provides a lifecycle node that subscribes to
a ``nav_msgs/OccupancyGrid``, classifies named regions, and reports the results
in the vocabulary of the loaded task.

Classification
~~~~~~~~~~~~~~

A region is a union of boxes, in metres in the map frame; cell coordinates are
not used because the grid is re-dimensioned as the map grows. A region's state
is determined by its cells (:numref:`tab-region`).

.. _tab-region:

.. list-table:: Region classification.
   :header-rows: 1
   :widths: 20 80

   * - State
     - Condition
   * - ``Blocked``
     - some cell is occupied
   * - ``Clear``
     - every cell has been observed and is free
   * - ``Unknown``
     - otherwise

Occupied takes precedence over unobserved, and unobserved over free. A cell
whose occupancy lies between the free and occupied thresholds counts as
unobserved, so that no agent is credited with knowledge it does not have.

Each region names the atom its state decides and the polarity
(``atom_true_when_clear``), so that the fleet tutorial's corridor maps to
``blocked`` without an additional atom ``clear_corridor``:

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

Reporting
~~~~~~~~~

A region bound to a sensing action (``sensing_action``) reports through
``apply_action`` with the configured outcome: the observation belongs to an
action the policy branches on. An unbound region is reported through
``announce``, as information from outside the plan.

Reports are sent on change only. A repeated announcement leaves the model
unchanged, but a sensing action applied twice does not, and both routes follow
the same rule. A transition back to ``Unknown`` sends nothing, since the state
has no operation that retracts knowledge.

Lifecycle
~~~~~~~~~

Both launch files start the node when ``epistemic_perception:=True``, as a
sixth managed node. It requires ``epistemic_state:=True``; the combination
without it is rejected at launch. ``plansys2_bringup`` manages it by name and
has no build dependency on it.

Regions are declared at configure time, when the parameter listing them has
been read. Since configure can be reached more than once, each region's
parameters are declared only if absent, and ``on_cleanup`` releases the
subscription and service client. Boxes are accepted as integer or double
arrays, as ``[2, -1, 8, 1]`` in YAML is an integer list.

.. _arch-executor:

Executor configuration
----------------------

Two executor parameters enable epistemic execution:

.. code-block:: yaml

   executor:
     ros__parameters:
       bt_builder_plugin: "EpistemicBTBuilder"
       bt_node_plugins: ["libplansys2_epistemic_bt_nodes.so"]

``bt_node_plugins`` is generic: the executor loads the named BehaviorTree.CPP
libraries before building a tree. ``plansys2_executor`` loads the epistemic
nodes at run time without linking them, and the nodes read the policy from
the blackboard entry in which the executor publishes the current plan.

.. _arch-packages:

Package dependencies
--------------------

.. list-table:: Epistemic packages and their dependency on the executor.
   :header-rows: 1
   :widths: 34 46 20

   * - Package
     - Contents
     - Depends on ``plansys2_executor``
   * - ``plansys2_epddl_grounder``
     - EPDDL front end: runs plank, caches the result
     - no
   * - ``plansys2_epistemic_msgs``
     - Nine service definitions
     - no
   * - ``plansys2_epistemic_planner``
     - Solver plugin over the Aletheia core; policy and belief serialisation
     - no
   * - ``plansys2_epistemic_executor``
     - Policy, tree rendering, five BT nodes, ``epistemic_state``
     - no
   * - ``plansys2_epistemic_perception``
     - Map regions and their reports to the state
     - no
   * - ``plansys2_epistemic_bt_builder``
     - ``BTBuilder`` plugin
     - yes
   * - ``plansys2_aletheia_plan_solver``
     - Subprocess plan solver plugin
     - no
   * - ``plansys2_tui_cli``
     - ``ros2 plansys2`` verbs and dashboard
     - no
   * - ``eplansys``
     - Metapackage
     - no

The builder is a separate package because it alone requires the executor's
plugin API; the remaining packages build and test against a released ROS
distribution as well as against this fork.

Within ``plansys2_epistemic_executor``, the policy and its rendering form a
separate library, ``plansys2_epistemic_executor_policy``, that depends only on
the message package and is tested as a pure transformation without ROS.
