Open-RMF execution
==================

``eplansys_rmf_bridge`` executes epistemic policies on a robot fleet managed by
Open-RMF :cite:p:`openrmf`. It is an ``ActionExecutorClient``: it accepts a
dispatched action, submits the corresponding RMF task, waits for completion,
and calls ``finish()`` with the outcome the robot observed, which the policy
branches on (:ref:`arch-observation`).

The bridge is maintained in a separate repository,
`eplansys-rmf <https://github.com/ePlanSys/eplansys-rmf>`_, so that
installations without a fleet do not depend on Open-RMF. The same arrangement
applies to the external planner binary of ``plansys2_aletheia_plan_solver``.

.. _fig-rmf-pipeline:

.. figure:: img/pipeline.svg
   :width: 100%
   :alt: EPDDL is grounded by plank and planned into a branching policy; the
         executor dispatches an action to eplansys_rmf_bridge, which submits an
         RMF task; the robot executes it and the observation returns as an
         outcome that updates the epistemic state.

   Position of the bridge between the policy and the fleet.

The bridge is the only component that both dispatches into the fleet and
receives the fleet's observations (:numref:`fig-rmf-pipeline`). Components
above it belong to the planning stack, components below it to Open-RMF. The
``outcome`` edge carries observations back into the epistemic state; without
it the bridge would only issue commands.

Division of responsibility
--------------------------

Open-RMF has no representation of knowledge; ePlanSys has no representation of
shared physical resources.

.. list-table:: Decisions made by each system.
   :header-rows: 1
   :widths: 22 78

   * - System
     - Determines
   * - ePlanSys
     - what must be found out, which agent acts, and which agents may learn
       the result
   * - Open-RMF
     - allocation of floor space, lifts and doors; robot availability

Performers
----------

The bridge creates one performer per action in its task map
(:doc:`task_map`). On dispatch, a performer behaves according to its entry:

* A movement action submits an RMF task pinned to the robot bound to the
  acting agent, waits for completion, and finishes.
* A ``local`` action is a speech act: no task is submitted, and the performer
  finishes after its declared duration. Its epistemic effect is computed by the
  epistemic state.
* A ``sensing`` action finishes with an outcome token, on which the policy
  branches.

All performers run in one process, ``rmf_action_node``, because they share the
websocket server the fleet adapter connects to and a port can be bound once.
They are spun by one ``MultiThreadedExecutor``.

Execution of one action
-----------------------

:numref:`fig-rmf-sequence` traces a movement action from dispatch to
completion.

.. _fig-rmf-sequence:

.. graphviz::
   :caption: A movement action through the bridge. Solid arrows are ROS 2
      topics; the dashed arrow is the fleet adapter's websocket.

   digraph rmf_sequence {
     rankdir=LR;
     node [shape=box, fontname="sans-serif", fontsize=10];
     edge [fontname="sans-serif", fontsize=9];

     executor  [label="plansys2 executor"];
     performer [label="performer\n(rmf_action_node)"];
     client    [label="RmfTaskClient"];
     tm        [label="robot TaskManager\n(fleet adapter)"];
     robot     [label="robot", shape=ellipse];

     executor  -> performer [label="1  actions_hub:\nrequest, confirm"];
     performer -> client    [label="2  agent → fleet/robot,\nzone → waypoint"];
     client    -> tm        [label="3  task_api_requests:\nrobot_task_request"];
     tm        -> client    [label="4  task_api_responses:\ntask id"];
     tm        -> robot     [label="5  navigation, traffic,\ndoors, lifts"];
     tm        -> client    [label="6  task state and log\nupdates", style=dashed];
     client    -> performer [label="7  phase, outcome token"];
     performer -> executor  [label="8  finish(success,\noutcome)"];
   }

1. The executor offers the action on ``actions_hub``; the performer for that
   action name accepts it.
2. The performer takes the acting agent from the first argument and looks up
   its robot in the ``agents`` table, and the destination as described in
   :doc:`task_map`. An agent without a binding fails the action at this point.
3. ``RmfTaskClient`` publishes a ``robot_task_request`` of category
   ``go_to_place`` for that fleet and robot. The request carries the labels
   ``eplansys.action=<name>`` and ``eplansys.agent=<agent>``. Publication is
   deferred until the robot has appeared on ``fleet_states``
   (:doc:`building`).
4. RMF answers on ``task_api_responses`` with the task id, which the client
   associates with its own request id.
5. The robot's ``TaskManager`` executes the task. Navigation, traffic
   negotiation, doors and lifts are handled by Open-RMF.
6. Task states and logs reach the bridge over the websocket named by the
   adapter's ``server_uri`` (:doc:`decisions`).
7. The client maps the RMF status to one of five phases (:doc:`building`) and
   extracts any value with the outcome prefix.
8. The performer calls ``finish()``. ``Succeeded`` finishes with success and,
   for a sensing action, the resolved outcome (:ref:`rmf-outcome-resolution`).
   ``Rejected``, ``Failed``, or no completion within ``task_timeout``
   (default 120 s) finish with failure.

A failed action fails the running plan. The executor's response is the
ordinary PlanSys2 one, and a mission run under ``MissionSupervisor`` plans
again from the belief the attempt reached (:ref:`arch-supervisor`).

Actions that move several robots
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

An action whose map entry declares ``movements`` submits one task per moved
agent in the same step. It succeeds when every task has succeeded and fails as
soon as one of them fails, since the model has no world in which half of a
joint move took place.

.. _rmf-outcome-resolution:

Outcome resolution
------------------

A sensing action's outcome can come from three sources besides the task map.
The bridge tries them in a fixed order (:numref:`tab-rmf-outcome`).

.. _tab-rmf-outcome:

.. list-table:: Sources of a sensing action's outcome, in order of precedence.
   :header-rows: 1
   :widths: 18 82

   * - Source
     - Condition
   * - fleet
     - The task state or log carried a value with the outcome prefix
       (``eplansys.outcome=`` by default).
   * - place
     - A message ``<place> <outcome>`` on the observation topic named this
       action's site. The site is the argument selected by ``outcome_arg``,
       else by ``waypoint_arg``, before translation through ``zones``.
   * - placeless
     - A message ``<outcome>`` with no place arrived at or after the start of
       the action.
   * - task map
     - ``outcomes`` or ``default_outcome`` names one. Every use is logged as a
       warning.
   * - none
     - The action finishes without an outcome and the error is logged; the
       policy has nothing to branch on.

The observation topic (``/eplansys/observation`` by default) is latched with a
depth of 16, so that a performer subscribing late still receives one reading
per site. Readings are kept by place. Timing alone cannot match them: a robot
usually reaches its site during the preceding ``goto``, so the correct reading
can be older than the scan that reports it. A reading for another site is
never used, and the bridge logs which sites it holds readings for.

Speech acts
-----------

In the survey domain the public and the private announcement differ only in
their audience. A map entry for a speech act declares a ``channel``, and the
bridge publishes the utterance on a topic whose subscribers form that audience:

.. list-table:: Speech-act channels.
   :header-rows: 1
   :widths: 18 40 42

   * - ``channel``
     - Topic
     - Audience
   * - ``public``
     - ``<channel_prefix>/public``
     - every agent
   * - ``private``
     - ``<channel_prefix>/private/<listener>``
     - the agent named by argument ``listener_arg`` (default 1)

``channel_prefix`` defaults to ``/eplansys/channel``. A publisher is created on
first use, so a topic appears on the graph only if the mission used it.

The utterance carries the outcome the speaker's own sensing action reported.
The action mapping of ``eplansys`` sends ``relay-dirty`` and ``relay-clean``
to the same PlanSys2 action ``(relay ?i ?j)``, so the performer cannot recover
from its arguments which of the two it executes; the bridge keeps the
speaker's last sensed outcome for this purpose.

This provides addressing. It does not provide confidentiality: any node may
subscribe to a private topic. What a run establishes is that the team used a
channel the observer was not on. Enforcing that the observer could not listen
requires DDS partitions or SROS 2 permissions, which a deployment can derive
from the same audiences.

Concurrent robots
-----------------

With ``parallel_dispatch`` enabled in the epistemic builder, independent runs
of a policy are dispatched as one ``Parallel`` group (:ref:`arch-bt`). Over
RMF this submits several ``go_to_place`` tasks at the same instant, and Open-RMF
routes the robots concurrently, with its traffic management resolving their
paths. On the ``rmf_demos`` office fleet, the two-site survey with the north
site dirty and the south site clear measured:

.. list-table:: Two-site survey over the office fleet.
   :header-rows: 1
   :widths: 50 50

   * - Dispatch
     - Mission time
   * - one policy node at a time
     - 67.5 s
   * - independent runs together
     - 51.5 s

The difference is one drive across the office: with sequential dispatch the
second robot departs after the first has arrived, scanned and reported.

Link interruption
-----------------

Publishing ``false`` on ``/eplansys/rmf_link`` (parameter ``link_topic``)
makes the bridge discard what arrives over the websocket from then on. The
socket stays connected, frames are still counted, and requests are still
published; only the return path is lost. ``true`` restores it. The topic is
latched, so an outage announced before the bridge starts still applies.

The mechanism exists for experiments on loss of feedback. An action under an
outage times out after ``task_timeout``, and a replan finds the fleet still
running. Terminating the fleet adapter measures a different failure: on
restart it assumes its robots are at their chargers, and tasks it accepts
afterwards do not complete.

Interfaces
----------

.. list-table:: Interfaces of ``rmf_action_node``.
   :header-rows: 1
   :widths: 34 16 50

   * - Name
     - Kind
     - Use
   * - ``actions_hub``
     - topic
     - PlanSys2 action protocol with the executor.
   * - ``task_api_requests``
     - topic, out
     - ``robot_task_request`` for each movement.
   * - ``task_api_responses``
     - topic, in
     - Task id for each request.
   * - ``fleet_states``
     - topic, in
     - Presence of each robot before its first request.
   * - ``ws://<host>:<websocket_port>``
     - websocket server
     - Task states and logs from the fleet adapter (default port 7879).
   * - ``/eplansys/observation``
     - topic, in
     - Perception readings (parameter ``observation_topic``).
   * - ``/eplansys/channel/...``
     - topics, out
     - Speech acts (parameter ``channel_prefix``).
   * - ``/eplansys/rmf_link``
     - topic, in
     - Link interruption (parameter ``link_topic``).

The remaining parameters, ``task_map``, ``websocket_port``,
``outcome_prefix`` and ``task_timeout``, are listed in :doc:`building`.

Reference scenario
------------------

The site survey (:doc:`../demos/survey`): three agents, a site that may be
contaminated, and a goal requiring that ``scout`` and ``relay`` learn the
finding and ``observer`` does not. The planner selects the private channel,
since a broadcast would falsify the third conjunct. :doc:`demo` runs this
mission over an RMF fleet.

A second scenario, the two-site survey (:doc:`../demos/two_sites`), runs the
same mission over two sites with one scout each, bound to the two robots of the
office fleet (``config/office_sites.json``). Its two halves share no agent,
atom or precondition, and both robots move.

After a demo run, ``mission_check`` evaluates the goal's conjuncts against the
epistemic state through ``epistemic_state/check_formula``, and reads the
transcripts of ``radio``, a node that records what each agent received on the
speech-act channels. The first check concerns the model; the second concerns
what was published and received. The second matters for the negative
conjunct: ``observer`` is bound to no robot and never moves, so
:math:`\neg\mathit{Kw}_{\mathit{observer}}` would also hold in a run that
ignored the observer entirely, and an empty transcript shows the observer was
on no channel the team used.

Verification
------------

The ``eplansys-rmf`` CI runs four jobs on pushes to ``main`` and on pull
requests (:numref:`tab-rmf-ci`). None of them starts Gazebo or a real fleet.

.. _tab-rmf-ci:

.. list-table:: CI of ``eplansys-rmf``.
   :header-rows: 1
   :widths: 26 74

   * - Job
     - Content
   * - build and test
     - Build of the bridge library without ``eplansys``; unit tests of the
       task map (19) and of outcome resolution (12).
   * - websocket smoke test
     - ``state_probe`` driven as a fleet adapter would drive it, including the
       bare ``Hello`` frame; checks that an outcome token is reported.
   * - mission end to end
     - The real ``rmf_action_node`` between a scripted executor on
       ``actions_hub`` and a scripted fleet adapter (a subscriber to
       ``task_api_requests`` and a websocket client), built against
       ``eplansys`` 0.3.0.
   * - lint
     - ``ament_uncrustify`` and ``ament_copyright``.

The end-to-end job asserts five properties:

#. a movement action is sent to the robot its agent is bound to, at the
   waypoint the map names;
#. a sensing action finishes with the outcome the fleet reported;
#. with the fleet silent, it finishes with the reading for its own site, and
   a reading published for another site is not used;
#. a private speech act reaches its listener's channel only, carrying what
   the speaker sensed;
#. an action whose agent is bound to no robot fails.

Scope
-----

What the bridge has been run against, and what it does not do:

* **Fleets.** Runs have used the ``rmf_demos`` office fleet in Gazebo on
  ROS 2 Humble, and the warehouse fleet of :doc:`warehouse`. All documented
  runs are in simulation.
* **Task category.** Only ``go_to_place`` is submitted. Actions that need
  other RMF task types, such as delivery or cleaning, are not supported.
* **Allocation.** Every task is pinned to the robot the planner named, and
  RMF's dispatcher does not bid. Assigning abstract roles and relabelling the
  epistemic agent after dispatch is not implemented (:doc:`decisions`).
* **Sensing.** The office fleet's adapter writes no outcome token, so in the
  demos the outcome comes from a perception reading or from the task map's
  stand-in, and each fallback is logged. An adapter that writes the token, as
  in :doc:`demo`, takes precedence.
* **Speech acts.** The bridge provides addressing without access control,
  as described above. A speech act moves no robot and submits no task.
* **Dependencies.** The bridge requires ``rmf_ros2`` (released for Humble);
  the demos additionally require ``rmf_demos`` built from source
  (:doc:`building`).

.. toctree::
   :maxdepth: 1

   decisions
   task_map
   demo
   warehouse
   diagnostics
   building
