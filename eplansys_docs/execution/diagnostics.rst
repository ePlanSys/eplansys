Diagnostics
===========

``eplansys_rmf_probe`` contains two programs that test the bridge's two
interfaces against a real fleet independently of any ``ActionExecutorClient``.
``state_probe`` tests the receiving side, using the bridge's ``WebsocketFeed``;
``submit_probe`` tests the submitting side.

state_probe
-----------

A websocket server that reads task states and logs and reports values carrying
a given prefix, i.e. whether an outcome token reaches the bridge.

.. code-block:: bash

   ros2 run eplansys_rmf_probe state_probe --port 7879

.. list-table:: ``state_probe`` options.
   :header-rows: 1
   :widths: 28 72

   * - Option
     - Meaning
   * - ``--port``
     - Listening port.
   * - ``--outcome-prefix``
     - Prefix identifying an outcome value.
   * - ``--all-logs``
     - Print every log entry.
   * - ``--raw``
     - Print frames as received.

The adapter is pointed at the probe with the panel disabled:

.. code-block:: bash

   ros2 launch eplansys_rmf_demo office_fleet.launch.xml \
       use_rmf_panel:=false server_uri:="ws://localhost:7879"

submit_probe
------------

Submits one task pinned to a named robot and prints its booking id.
``state_probe`` groups its reports by the same id, which the bridge uses to
match a completed RMF task to the epistemic action that requested it.

.. code-block:: bash

   ros2 run eplansys_rmf_probe submit_probe -F tinyRobot -R tinyRobot1 \
       -p patrol_A1

.. list-table:: ``submit_probe`` options.
   :header-rows: 1
   :widths: 28 72

   * - Option
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
     - Submit a ``dispatch_task_request``, leaving robot selection to RMF's
       dispatcher, for comparison with the pinned path.
   * - ``--timeout``
     - Response timeout.

Output
------

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

A stock ``go_to_place`` task carries no outcome, and the summary reports none.
:doc:`demo` shows how a fleet adapter writes the token.

Launch files that start ``rmf_demos_panel`` may require ``PYTHONNOUSERSITE=1``,
as a ``pip --user`` installation of Werkzeug can shadow the panel's Flask. The
probes are unaffected.
