Design decisions
================

Allocation authority
--------------------

Both systems allocate. The planner assigns actions to named agents, and the
epistemic model records knowledge against those names; the RMF dispatcher
collects bids and selects a robot. If the two choices differ, the model credits
an agent with knowledge it never sensed, and no error is raised.

The bridge therefore binds each RMF task to the robot of the agent the planner
named, through the ``agents`` table of the task map. It submits a
``robot_task_request``, which names a fleet and a robot and is handled directly
by that robot's ``TaskManager`` without bidding. Traffic management, lifts and
doors remain under RMF; only RMF's allocation is bypassed.

An action whose agent is bound to no robot is refused. Delegating the choice to
RMF would reintroduce the silent substitution this decision prevents.

A role-based scheme, in which the planner reasons over abstract agents and the
bridge relabels the epistemic agent after dispatch, remains a research
question; relabelling an agent in a Kripke model during execution is
non-trivial.

Outcome transport
-----------------

An RMF task carries no result payload. This was established from the sources,
as the Open-RMF documentation does not address it:

* ``task_state.json`` in ``rmf_api_msgs`` has no result, return or output
  property at any level. Its completion fields are a status from a fixed
  enumeration and a finish time.
* ``RobotUpdateHandle::ActionExecution::finished()``, through which a
  performer would report, takes no argument.
* The ``DynamicEvent`` action result carries a failure string, a status string
  and an event id, and no domain data.

Two free-form fields travel with a task, and the bridge reads both. A value
with the prefix ``eplansys.outcome=`` in either is taken as the outcome token
(:numref:`tab-outcome-routes`).

.. _tab-outcome-routes:

.. list-table:: Routes for the outcome token.
   :header-rows: 1
   :widths: 22 40 38

   * - Route
     - Written through
     - Requirement
   * - event ``detail``
     - ``SimpleEventState::update_detail``, forwarded verbatim in every state
       update
     - a custom ``rmf_task_sequence`` event
   * - log entries
     - ``underway()`` and related calls
     - an ordinary ``perform_action`` callback

In a fleet adapter, the token is written with one call on the execution handle:

.. code-block:: python

   execution.underway("eplansys.outcome=e-scan-dirty")

On Humble, task states and logs leave the fleet adapter only over the websocket
named by the adapter's ``server_uri`` parameter. ``StandardNames.hpp`` declares
only ``task_api_requests`` and ``task_api_responses``; the ROS 2 topics
``task_state_update`` and ``task_log_update`` exist on Rolling only. The bridge
is therefore the websocket server the adapter connects to.

Action-to-task mapping
----------------------

``action_mapping.json`` in ``eplansys`` maps grounded action names to PlanSys2
action expressions and is applied inside the planner. A performer receives only
the expression (a name and arguments), so a map keyed on grounded names cannot
be consulted there.

The bridge uses a separate map, keyed on PlanSys2 action names, that binds
agents to robots and gives each action's destination (:doc:`task_map`).

Websocket server
----------------

``rmf_websocket::BroadcastServer`` is unsuitable as the feed. With version
2.1.8 an adapter reports the connection open and publishes without error, but
the server's callback is never invoked; a plain websocket server on the same
port receives every frame from the same adapter. The server also aborts the
process on the first non-JSON frame: its handler calls
``nlohmann::json::parse`` with exceptions enabled, and ``BroadcastClient``
opens with a bare ``Hello`` probe.

``WebsocketFeed`` is the bridge's own server; it ignores unparseable frames.
