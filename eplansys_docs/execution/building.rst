Building and testing
====================

Dependencies
------------

Open-RMF is released into Humble in full, so this is binaries rather than a
source build of ``rmf_ros2``. ``websocketpp`` and ``boost::system`` are for
``WebsocketFeed``.

.. code-block:: bash

   sudo apt install ros-humble-rmf-dev libwebsocketpp-dev libboost-system-dev
   colcon build

``eplansys_rmf_bridge`` looks for plansys2 with ``QUIET`` and builds its library
without it, so the RMF half compiles and its tests run on a machine that has
Open-RMF and no ``eplansys``. The performers need both. ``rmf_demos`` is not
released into Humble and has to be built from source for the demo; its
``humble`` branch is the one to use.

Node parameters
---------------

.. list-table::
   :header-rows: 1
   :widths: 26 26 48

   * - Parameter
     - Default
     - Meaning
   * - ``task_map``
     - ``""``
     - Path to the task map JSON.
   * - ``websocket_port``
     - ``7879``
     - Port the fleet adapter is told to dial.
   * - ``outcome_prefix``
     - ``eplansys.outcome=``
     - Marks a detail or log value as an epistemic outcome.
   * - ``task_timeout``
     - ``120.0``
     - Seconds before a submitted task is given up on.

Task phases
-----------

``RmfTaskClient::status`` reports one of five phases against the request id
returned by submission. The RMF task id is not known until the response arrives,
which is why the request id is the handle.

.. list-table::
   :header-rows: 1
   :widths: 22 78

   * - Phase
     - Meaning
   * - ``Pending``
     - Submitted, no ``ApiResponse`` yet.
   * - ``Rejected``
     - RMF refused it; the reason is in ``error``.
   * - ``Running``
     - Accepted and under way.
   * - ``Succeeded``
     - Reached ``completed``.
   * - ``Failed``
     - Reached failed, canceled or killed.

Submission is deferred until the robot appears. A performer can be dispatched
its first action within a second of the process starting, well before the fleet
adapter is up, and a ``robot_task_request`` published then is simply lost: only
the named robot's own ``TaskManager`` handles one, the dispatcher ignores it,
and no error is raised anywhere. The request therefore waits until that robot
has announced itself on ``fleet_states``, which is the earliest point at which
its adapter is known to be running and discovered.

Tests
-----

The unit tests cover the task map. The websocket is covered by a smoke test that
drives ``state_probe`` the way a fleet adapter does and checks that an outcome
token comes back out, including the bare ``Hello`` frame that used to take the
process down.

.. code-block:: bash

   colcon test --packages-select eplansys_rmf_bridge
   colcon test-result --verbose
   source install/setup.bash
   .github/workflows/smoke_test.sh

Code style
----------

The tree follows the ament default that ``ament_uncrustify`` enforces: K&R
bracing and ``const auto & x`` spacing. Open-RMF itself is written in Allman
style, so code adapted from ``rmf_ros2`` will not match and CI will reject it.
Copyright notices go in ``//`` line comments; ``ament_copyright`` does not
recognise the block form and reports the file as having no notice at all.

.. code-block:: bash

   ament_uncrustify eplansys_rmf_bridge eplansys_rmf_probe
   ament_copyright eplansys_rmf_bridge eplansys_rmf_probe

``--reformat`` fixes the first in place. CI runs both, plus the build, the unit
tests and the smoke test, against ``ros:humble-ros-base-jammy``. ``eplansys`` is
deliberately absent from that image, so what CI exercises is the whole RMF half
and none of the performers.
