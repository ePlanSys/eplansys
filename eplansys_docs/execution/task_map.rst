The task map
============

The task map is a JSON file passed to the bridge as the ``task_map`` parameter
and read once at start-up. ``TaskMapping::load`` throws
``std::runtime_error`` if the file cannot be read or does not have the
structure below.

.. list-table:: Top-level tables.
   :header-rows: 1
   :widths: 18 82

   * - Table
     - Contents
   * - ``zones``
     - Domain zone name → RMF waypoint name. Unlisted zones pass through
       unchanged, so a map may use RMF waypoint names directly.
   * - ``agents``
     - Epistemic agent name → ``{"fleet": ..., "robot": ...}``. An unlisted
       agent cannot act.
   * - ``actions``
     - PlanSys2 action name → specification. Each key yields one performer.

Action fields
-------------

.. list-table:: Fields of an action specification.
   :header-rows: 1
   :widths: 22 14 64

   * - Field
     - Default
     - Meaning
   * - ``local``
     - ``false``
     - Speech act: no task is submitted; completes after ``duration``.
   * - ``duration``
     - ``1.0``
     - Duration of a local action, in seconds.
   * - ``category``
     - ``go_to_place``
     - RMF task category. Only ``go_to_place`` is supported.
   * - ``waypoint``
     - none
     - Fixed destination.
   * - ``waypoints``
     - ``{}``
     - Per-agent destinations overriding ``waypoint``, e.g. for several agents
       scanning different places under one action name.
   * - ``waypoint_arg``
     - ``-1``
     - Index of the argument naming the destination zone; negative selects the
       fixed destination.
   * - ``movements``
     - ``[]``
     - Robots moved by one action (below).
   * - ``orientation``
     - none
     - Final orientation in radians.
   * - ``sensing``
     - ``false``
     - The action senses; only a sensing action's ``finish()`` carries an
       outcome.
   * - ``outcome_arg``
     - ``-1``
     - Index of the argument whose value selects the outcome, for a sensing
       action performed in several places.
   * - ``outcomes``
     - ``{}``
     - Argument value → outcome token, used when ``outcome_arg`` is set.
   * - ``default_outcome``
     - ``""``
     - Outcome reported when RMF carries no token. Empty: a missing token
       fails the action.

Destination resolution
----------------------

``waypoint_for`` resolves a destination from the specification, the acting
agent and the action's arguments, in the order of
:numref:`fig-resolve`, and translates the result through ``zones``.

.. _fig-resolve:

.. graphviz::
   :caption: Precedence of destination sources. Unlisted zones pass through
      untranslated.

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

Multi-robot actions
-------------------

The executor runs a policy's actions in order, so two consecutive movement
actions move two robots one after the other. Simultaneous movement must be
modelled as one event that relocates several agents. ``movements`` lists, for
such an action, which arguments name the agents and their destinations.

.. list-table:: Fields of a ``movements`` entry.
   :header-rows: 1
   :widths: 22 14 64

   * - Field
     - Default
     - Meaning
   * - ``agent_arg``
     - ``0``
     - Index of the argument naming the moving agent.
   * - ``waypoint_arg``
     - ``1``
     - Index of the argument naming its destination zone.

An empty ``movements`` denotes an action that moves only the acting agent,
whose destination ``waypoint_arg`` and ``waypoint`` describe.

Simulated outcomes
------------------

``outcomes`` encodes the simulator's ground truth, e.g. which suite is flooded,
so that a demonstration on a fleet without the relevant sensor can execute both
branches of its policy. A robot with a real sensor reports its measurement, and
``outcomes`` is not consulted.

``default_outcome`` serves the same purpose for fleet adapters unaware of
ePlanSys, which write no token. Each fallback is logged as a warning. A
deployment leaves it empty, so that a missing token fails the action.

Example
-------

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

An action whose destination is an argument, such as
``(goto_zone inspector lobby l3_suite)``, sets ``"waypoint_arg": 2``. A sensing
action performed in several places, such as ``(look_into inspector l2_suite)``,
sets ``"outcome_arg": 1`` and lists the tokens under ``outcomes``.
