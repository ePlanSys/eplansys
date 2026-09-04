The task map
============

A JSON file, given to the bridge as the ``task_map`` parameter. It has three
top-level tables. It is read once at startup, and ``TaskMapping::load`` throws
``std::runtime_error`` if the file cannot be read or is not shaped as described
here.

.. list-table::
   :header-rows: 1
   :widths: 18 82

   * - Table
     - Contents
   * - ``zones``
     - domain zone name to RMF waypoint name. A zone with no entry is passed
       through unchanged, so a map may name RMF waypoints directly.
   * - ``agents``
     - epistemic agent name to ``{"fleet": ..., "robot": ...}``. An agent
       absent from this table cannot act.
   * - ``actions``
     - PlanSys2 action name to its specification. Every key here becomes one
       performer.

Action fields
-------------

.. list-table::
   :header-rows: 1
   :widths: 22 14 64

   * - Field
     - Default
     - Meaning
   * - ``local``
     - ``false``
     - A speech act. Submits no task and completes after ``duration``.
   * - ``duration``
     - ``1.0``
     - Seconds a local action takes. Purely for the look of the thing.
   * - ``category``
     - ``go_to_place``
     - RMF task category. Only ``go_to_place`` is handled today.
   * - ``waypoint``
     - none
     - The fixed destination.
   * - ``waypoints``
     - ``{}``
     - Per-agent overrides of ``waypoint``. This is what lets three agents scan
       three different places under one action name.
   * - ``waypoint_arg``
     - ``-1``
     - Index of the argument naming the zone, for an action whose destination
       varies. Negative means use the fixed waypoint.
   * - ``movements``
     - ``[]``
     - For an action that moves several robots at once.
   * - ``orientation``
     - none
     - Final orientation, in radians, if the task should set one.
   * - ``sensing``
     - ``false``
     - The action finds something out. Only a sensing action's ``finish()``
       carries an outcome.
   * - ``outcome_arg``
     - ``-1``
     - Index of the argument whose value selects the outcome, for a sensing
       action performable in several places.
   * - ``outcomes``
     - ``{}``
     - Argument value to outcome token, consulted when ``outcome_arg`` is set.
   * - ``default_outcome``
     - ``""``
     - Reported when the action senses and RMF carried no token. Empty means an
       absent token fails the action.

How a destination is resolved
-----------------------------

``waypoint_for`` takes the action's specification, the acting agent and the
action's arguments, and consults three sources in a fixed order. The result is
then translated through ``zones``.

.. graphviz::
   :caption: Precedence when resolving a destination; an unlisted zone passes through untranslated

   digraph resolve {
     rankdir=LR;
     node [shape=box, fontname="sans-serif", fontsize=10];
     edge [fontname="sans-serif", fontsize=9];

     arg   [label="waypoint_arg"];
     per   [label="waypoints[agent]"];
     fixed [label="waypoint"];
     zones [label="zones"];
     out   [label="RMF waypoint"];

     arg   -> zones [label="1st"];
     per   -> zones [label="2nd"];
     fixed -> zones [label="3rd"];
     zones -> out   [label="translate"];
   }

Actions that move several robots
--------------------------------

A policy is a chain of product updates and the executor runs it strictly in
order, so two consecutive move actions are two robots moving one after the
other. Robots move together only when the togetherness is in the model: one
event, applied once, that relocates several agents. ``movements`` says which of
that action's arguments name the agents and where each of them goes.

.. list-table::
   :header-rows: 1
   :widths: 22 14 64

   * - Field
     - Default
     - Meaning
   * - ``agent_arg``
     - ``0``
     - Index of the argument naming the agent that moves.
   * - ``waypoint_arg``
     - ``1``
     - Index of the argument naming the zone it moves to.

An empty ``movements`` is the ordinary case of an action that moves its acting
agent alone, which ``waypoint_arg`` and ``waypoint`` then describe.

Outcomes on a fleet with no sensor
----------------------------------

``outcomes`` is the simulator's ground truth and nothing else: which suite is
actually flooded. A robot with a real sensor reports what it measured and none
of it is consulted. It exists so that a demo on a fleet without a water sensor
can still take both branches of its own policy.

``default_outcome`` serves the same end from the other direction. A fleet
adapter that knows nothing of ePlanSys writes no outcome at all, and naming the
answer here runs the mission end to end with RMF driving the robots. The bridge
logs a warning every time it falls back. Leaving it empty makes an absent token
fail the action, which is what a real deployment wants.

Example
-------

A map with a zone table, two bound agents and four actions:

.. code-block:: json

   {
     "zones": {
       "lobby": "lobby",
       "l2_suite": "L2_master_suite"
     },
     "agents": {
       "scout": {"fleet": "tinyRobot", "robot": "tinyRobot1"},
       "relay": {"fleet": "tinyRobot", "robot": "tinyRobot2"}
     },
     "actions": {
       "goto_site": {"category": "go_to_place", "waypoint": "pantry"},
       "scan":      {"sensing": true, "waypoint": "pantry",
                     "default_outcome": "e-scan-dirty"},
       "relay":     {"local": true, "duration": 2.0},
       "broadcast": {"local": true, "duration": 2.0}
     }
   }

An action whose destination comes from its arguments, such as
``(goto_zone inspector lobby l3_suite)``, sets ``"waypoint_arg": 2``. A sensing
action performable in several places, such as
``(look_into inspector l2_suite)``, sets ``"outcome_arg": 1`` and lists the
answers under ``outcomes``.
