Design decisions
================

Three questions the bridge had to answer, and what it settled on.

Allocation authority
--------------------

Both systems allocate, and their decisions need not coincide. The planner
assigns actions to named agents during search and the epistemic model records
knowledge against those names; the RMF dispatcher receives bids and selects a
robot. Where the two differ, the model records that an agent knows something it
never sensed, and no error is raised.

The bridge therefore binds each RMF task to the robot the planner named,
through the ``agents`` table of its task map. The mechanism is
``robot_task_request``, which carries a fleet and a robot and is handled
directly by that robot's own ``TaskManager`` without a bid. Traffic management,
lifts and doors are retained in full; only RMF's allocation is forgone.

An agent bound to no robot is a hard error, and the bridge refuses the action.
Letting RMF choose instead is precisely the failure this decision exists to
prevent: a silent substitution would credit the wrong agent with sensing, and
nothing downstream would notice.

The role-based scheme, in which the planner reasons over abstract agents and
the bridge relabels the epistemic agent after dispatch, remains the research
variant. Relabelling an agent in a Kripke model during execution is not a
trivial operation.

Outcome transport
-----------------

Settled by reading the sources, since Open-RMF's documentation does not address
it: a task carries no result payload.

* ``rmf_api_msgs``' ``task_state.json`` has no result, return or output
  property at any level. Its completion-bearing fields are a status token from
  a fixed enumeration and a finish time.
* ``RobotUpdateHandle::ActionExecution::finished()`` takes no argument, which
  is the exact API a performer would report through.
* The newer ``DynamicEvent`` action result carries a failure string, a status
  string and an event id, and nothing of the domain.

Two free-form fields do travel with a task, and the bridge reads both. A value
carrying the prefix ``eplansys.outcome=`` in either is read as the token the
policy branches on.

.. list-table::
   :header-rows: 1
   :widths: 22 40 38

   * - Route
     - Written through
     - Trade-off
   * - event ``detail``
     - ``SimpleEventState::update_detail``, forwarded verbatim into every state
       update
     - tidier; needs a custom ``rmf_task_sequence`` event
   * - log entries
     - ``underway()`` and its neighbours
     - reachable from an ordinary ``perform_action`` callback

In a fleet adapter, writing the token is one line on the execution handle:

.. code-block:: python

   execution.underway("eplansys.outcome=e-scan-dirty")

On Humble this stream leaves the fleet adapter over the websocket named by the
adapter's ``server_uri`` parameter and over nothing else.
``StandardNames.hpp`` declares only ``task_api_requests`` and
``task_api_responses``; the ROS 2 mirror of ``task_state_update`` and
``task_log_update`` exists on Rolling and not here. The bridge is therefore the
websocket server the adapter dials.

Action to task mapping
----------------------

``eplansys``' ``action_mapping.json`` maps plank's grounded names to PlanSys2
action expressions, and does its work inside the planner. By the time an action
reaches a performer the grounded name is gone and what remains is a name and
arguments, so a map keyed on grounded names could not be consulted here. The
two files answer different questions at different times, and collapsing them
would leave the bridge unable to look anything up.

The bridge keeps a separate map, keyed differently: it binds agents to robots
and says where each action sends one. Its format is given in :doc:`task_map`.

Why not rmf_websocket::BroadcastServer
--------------------------------------

``rmf_websocket::BroadcastServer`` is the obvious thing to use for the feed and
does not work. Measured against 2.1.8, an adapter reports the connection open
and publishes without error while that server's callback is never invoked, and
a plain websocket server on the same port receives every frame from the same
adapter. It also aborts the process on the first non-JSON frame, since its
handler calls ``nlohmann::json::parse`` with exceptions enabled and
``BroadcastClient`` opens by sending a bare ``Hello`` probe.

``WebsocketFeed`` is therefore the bridge's own server, and it treats an
unparseable frame as a frame to ignore.
