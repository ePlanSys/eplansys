Diagnostics
===========

``eplansys_rmf_probe`` holds two programs that establish the bridge's two ends
against a real fleet before any ``ActionExecutorClient`` exists. ``state_probe``
is the receiving end, built on the same ``WebsocketFeed`` the bridge uses;
``submit_probe`` is the sending end.

state_probe
-----------

A websocket server that reads task states and task logs and reports any value
carrying a known prefix. That settles whether an epistemic outcome token
survives the trip.

.. code-block:: bash

   ros2 run eplansys_rmf_probe state_probe --port 7879

.. list-table::
   :header-rows: 1
   :widths: 28 72

   * - Flag
     - Meaning
   * - ``--port``
     - Port to listen on.
   * - ``--outcome-prefix``
     - Prefix that marks a value as an epistemic outcome.
   * - ``--all-logs``
     - Print every log entry, not only those carrying the prefix.
   * - ``--raw``
     - Print the frames as they arrive.

Point the adapter at it, with the panel switched off:

.. code-block:: bash

   ros2 launch eplansys_rmf_demo office_fleet.launch.xml \
       use_rmf_panel:=false server_uri:="ws://localhost:7879"

submit_probe
------------

Submits one task pinned to a named robot and prints the booking id it was
given. ``state_probe`` groups everything it reports under that same id, which is
the key the bridge uses to match a finished RMF task to the epistemic action
that asked for it.

.. code-block:: bash

   ros2 run eplansys_rmf_probe submit_probe -F tinyRobot -R tinyRobot1 \
       -p patrol_A1

.. list-table::
   :header-rows: 1
   :widths: 28 72

   * - Flag
     - Meaning
   * - ``-F``, ``--fleet``
     - Fleet name.
   * - ``-R``, ``--robot``
     - Robot name.
   * - ``-p``, ``--place``
     - Destination waypoint.
   * - ``-o``, ``--orient``
     - Final orientation.
   * - ``--dispatch``
     - Send a ``dispatch_task_request`` instead and let RMF's dispatcher choose
       the robot, for comparison against the pinned path.
   * - ``--timeout``
     - How long to wait for a response.

Reading the output
------------------

.. code-block:: text

   [11:32:59] task compose.dispatch-0: assigned to tinyRobot/tinyRobot1
   [11:32:59] task compose.dispatch-0: none -> underway
   [11:33:00] task compose.dispatch-0: OUTCOME via log: "e-scan-dirty"
   [11:33:01] task compose.dispatch-0: underway -> completed

     task     compose.dispatch-0
     category compose
     robot    tinyRobot/tinyRobot1
     status   completed
     outcome  "e-scan-dirty" (via log)

A stock ``go_to_place`` task carries no outcome, and the summary says so. To see
one, have the fleet adapter's action executor write the token as shown in
:doc:`demo`.

``PYTHONNOUSERSITE=1`` may be needed for anything that launches
``rmf_demos_panel``, whose Flask can be shadowed by a pip ``--user`` Werkzeug.
Neither probe is affected.
