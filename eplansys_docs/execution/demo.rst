Survey demo over Open-RMF
=========================

The site survey of :doc:`../demos/survey`, executed by an Open-RMF fleet. One
command starts the office fleet, the planning system and the bridge:

.. code-block:: bash

   ros2 launch eplansys_rmf_demo survey_rmf_launch.py
   ros2 launch eplansys_rmf_demo survey_rmf_launch.py site:=clean

The EPDDL problem and the policy are those of ``eplansys_demo``. The performers
submit RMF tasks and wait for the fleet to complete them. ``site`` selects the
outcome of the scan, and with it the branch: ``relay-dirty_relay_scout`` or
``relay-clean_relay_scout``.

.. list-table:: Launch arguments.
   :header-rows: 1
   :widths: 18 16 66

   * - Argument
     - Default
     - Meaning
   * - ``site``
     - ``dirty``
     - Scan outcome: ``dirty`` or ``clean``.
   * - ``headless``
     - ``false``
     - Run Gazebo headless, without rviz.
   * - ``rmf``
     - ``true``
     - Launch the office fleet. ``false`` expects it in another terminal.

Task map
--------

``config/office_survey.json`` binds each agent to a robot of the
``rmf_demos`` office fleet (:numref:`tab-office-agents`) and gives each
action's destination.

.. _tab-office-agents:

.. list-table:: Agent bindings.
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

``observer`` is unbound: the office fleet has two robots, and the policy never
moves ``observer``. If a modified goal led the planner to move it, the bridge
would refuse the action with an explanatory error.

``goto_site`` and ``scan`` both target ``pantry``. ``scan`` therefore
completes almost immediately, standing in for a sensing task; a deployment
would use a ``perform_action`` declared by the fleet. ``relay`` and
``broadcast`` are speech acts and submit no task.

Ports
-----

The bridge listens on port 7879 as the fleet adapter's websocket server. The
``rmf_demos`` panel uses port 7878 and is disabled, since an adapter has a
single ``server_uri`` (:numref:`fig-rmf-ports`).

.. _fig-rmf-ports:

.. graphviz::
   :caption: A fleet adapter feeds one destination.

   digraph ports {
     rankdir=LR;
     node [shape=box, fontname="sans-serif", fontsize=10];
     edge [fontname="sans-serif", fontsize=9];

     adapter [label="fleet adapter\n(one server_uri)"];
     bridge  [label="eplansys_rmf_bridge\nport 7879"];
     panel   [label="rmf_demos panel\nport 7878, disabled", style=dashed];

     adapter -> bridge [label="states, logs"];
     adapter -> panel  [style=dashed, label="not available"];
   }

``rmf_demos_gz_classic/office.launch.xml`` forwards neither ``server_uri`` nor
``use_rmf_panel`` to ``common.launch.xml``; setting them on its command line
has no effect and no warning. ``office_fleet.launch.xml`` assembles the same
demo from the same components and forwards both.

Outcome path
------------

``site`` sets the bridge's ``default_outcome``, used when the fleet reports no
token; each fallback is logged as a warning. To exercise the fleet-reported
path, the fleet adapter's action executor writes the token, e.g. in
``rmf_demos_fleet_adapter/fleet_adapter.py``:

.. code-block:: python

   execution.underway("eplansys.outcome=e-scan-dirty")

The bridge then reports the fleet's token and ignores the default.
