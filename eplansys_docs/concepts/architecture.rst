Architecture
============

ePlanSys adds four packages to PlanSys2 and changes none of its own. The
planner is a plan solver plugin, the tree builder is a behavior tree builder
plugin, the epistemic behavior tree nodes are loaded as a BehaviorTree.CPP
plugin library, and the model of what the agents know is a separate lifecycle
node. Nothing in ``plansys2_executor`` links against any of it.

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
     }

     planner  -> solver   [label="loads"];
     executor -> builder  [label="loads"];
     executor -> nodes    [label="loads"];
     executor -> problem  [label="facts"];
     executor -> domain   [label="action trees"];
     nodes    -> state    [label="services"];
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

All of its parameters are strings, declared under the plugin's own name at
configure time:

.. list-table::
   :header-rows: 1
   :widths: 22 18 60

   * - Parameter
     - Default
     - Meaning
   * - ``task_file``
     - empty
     - Absolute path to a grounded task JSON. Empty requires the task to
       arrive in the planning request instead.
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
     WaitAtStartReq         action="(peek A):0"    ┐
     ApplyAtStartEffect     action="(peek A):0"    │
     ReactiveSequence                              │ PlanSys2, untouched
       CheckOverAllReq      action="(peek A):0"    │
       ExecuteAction        action="(peek A):0"    │
     CheckAtEndReq          action="(peek A):0"    │
     ApplyAtEndEffect       action="(peek A):0"    ┘
     ApplyEpistemicUpdate   node="0" outcome="{epistemic_outcome_0}"
     EpistemicSwitch        node="0" outcome="{epistemic_outcome_0}"
                                     outcomes="e_tails;e_heads"
       Sequence "node_1 (shout-tails A)"
       Sequence "node_2 (open A)"
   CheckEpistemicGoal goal="(K A tails)"

A node with a single continuation renders without a switch, so a classical plan
comes out as the same flat sequence PlanSys2 would have built.

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
   ``action``, and ``observed``, the outcome the performer observed, which a
   domain-specific tree binds to whatever its performer reports and which is
   left empty to let the state decide. Output port: ``outcome``, the outcome
   that occurred, which the following ``EpistemicSwitch`` reads.

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
model at a time, for one grounded task, and answers three services.

.. list-table::
   :header-rows: 1
   :widths: 38 62

   * - Service
     - Question
   * - ``epistemic_state/load_task``
     - Be the model of this grounded task. The task is supplied inline as
       ``task_json`` or by path as ``task_file``, and loading resets the model
       to that task's initial state. The response reports the number of
       worlds, agents and actions loaded.
   * - ``epistemic_state/check_formula``
     - Does this formula hold now? The response distinguishes a failure, no
       task loaded or a formula that does not parse, from the answer itself.
   * - ``epistemic_state/apply_action``
     - This action ran: perform the DEL product update and report what was
       observed. The request names the action in the task's own vocabulary and
       may carry the observed outcome; the response reports the outcome that
       occurred and the size of the resulting state.

It also declares one parameter, ``task_file``, and publishes on
``epistemic_state/state`` with transient-local durability. A task named at
configure time is the common case: one mission, one task, loaded before
anything asks a question about it.

The state advances by executed actions rather than by watching the world, which
is what makes it a belief state rather than a log. When it disagrees with what
the robot observed, the disagreement surfaces at ``apply_action`` as an outcome
the model cannot account for: a reason to replan, not to overwrite the model
quietly.

Which outcome occurred is a question about the world, not about the model. When
the state designates a single world it already answers it, and the response
reports it. When it designates several it genuinely does not know, and the
observation has to come from whoever did the sensing.

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
   * - ``plansys2_epistemic_bt_builder``
     - The ``BTBuilder`` plugin
     - Yes

The builder is a separate package because it is the only piece that needs the
executor's plugin API. Keeping it apart lets everything else build and be
tested against a released distribution rather than only against this fork.

Within ``plansys2_epistemic_executor`` the same boundary is drawn once more.
The policy and its rendering as a behavior tree are built as a separate
library, ``plansys2_epistemic_executor_policy``, which depends on nothing but
the message package: they are the shared reading of a policy, and keeping them
free of ROS lets them be tested as the pure transformation they are.
