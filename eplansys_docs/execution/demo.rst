Running the survey demo
=======================

The site survey of ``eplansys``, run over an Open-RMF fleet. One command brings
up the office fleet, the planning system and the bridge.

.. code-block:: bash

   ros2 launch eplansys_rmf_demo survey_rmf_launch.py
   ros2 launch eplansys_rmf_demo survey_rmf_launch.py site:=clean

Same mission, same EPDDL, same policy as ``eplansys_demo``. What changes is who
moves the robots: those performers wait out a duration, and these submit RMF
tasks and wait for the fleet to report them done. ``site:`` chooses what the
scout turns out to find, and the policy takes a different branch for each,
``relay-dirty_relay_scout`` against ``relay-clean_relay_scout``, with the robot
driven by RMF either way.

.. list-table::
   :header-rows: 1
   :widths: 18 16 66

   * - Argument
     - Default
     - Meaning
   * - ``site``
     - ``dirty``
     - What the scout turns out to find: ``dirty`` or ``clean``.
   * - ``headless``
     - ``false``
     - Run Gazebo headless and leave rviz out.
   * - ``rmf``
     - ``true``
     - Launch the office fleet too. ``false`` leaves it to another terminal.

The map
-------

``config/office_survey.json`` binds each epistemic agent to one robot of the
``rmf_demos`` office fleet and says where each action sends it.

.. list-table::
   :header-rows: 1
   :widths: 30 70

   * - Agent
     - Robot
   * - ``scout``
     - ``tinyRobot1``
   * - ``relay``
     - ``tinyRobot2``
   * - ``observer``
     - none

``observer`` is deliberately unbound. The office fleet has two robots, and the
mission's third conjunct is that the observer must not come to know, so the
planner does not send it anywhere. If a change to the goal ever made the planner
choose to move it, the bridge refuses the action and says why, which is the
behaviour worth having.

``goto_site`` and ``scan`` both go to ``pantry``. ``scan`` names the waypoint
the robot has already reached, so RMF completes it almost at once; it stands in
for a sensing task, and a deployment would use a ``perform_action`` the fleet
declares. ``relay`` and ``broadcast`` are speech acts and submit nothing.

Ports
-----

The bridge is the websocket server the fleet adapter dials, on 7879. The
``rmf_demos`` panel owns 7878 and is switched off here, since an adapter has one
``server_uri`` and cannot feed both.

.. graphviz::
   :caption: An adapter feeds one destination, so the panel and the bridge cannot both receive the stream

   digraph ports {
     rankdir=LR;
     node [shape=box, fontname="sans-serif", fontsize=10];
     edge [fontname="sans-serif", fontsize=9];

     adapter [label="fleet adapter\n(one server_uri)"];
     bridge  [label="eplansys_rmf_bridge\nport 7879"];
     panel   [label="rmf_demos panel\nport 7878, switched off", style=dashed];

     adapter -> bridge [label="states, logs"];
     adapter -> panel  [style=dashed, label="not available"];
   }

``office_fleet.launch.xml`` exists because
``rmf_demos_gz_classic/office.launch.xml`` forwards neither ``server_uri`` nor
``use_rmf_panel`` down to ``common.launch.xml``, so setting them on its command
line does nothing, silently. It assembles the same demo from the same pieces and
passes both.

Seeing the real outcome path
----------------------------

``site:=dirty|clean`` sets the ``default_outcome`` the bridge falls back to when
the fleet reports none, and the fallback is logged as a warning every time. To
exercise the real path instead, have the fleet adapter's action executor write
the token. In ``rmf_demos_fleet_adapter/fleet_adapter.py`` that is one line on
the ``execution`` handle:

.. code-block:: python

   execution.underway("eplansys.outcome=e-scan-dirty")

The bridge then reports what the fleet observed and ignores the default.
