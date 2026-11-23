Building and testing
====================

Dependencies
------------

Open-RMF is released for Humble, so ``rmf_ros2`` is installed from binaries.
``websocketpp`` and ``boost::system`` are required by ``WebsocketFeed``.

.. code-block:: bash

   sudo apt install ros-humble-rmf-dev libwebsocketpp-dev libboost-system-dev
   colcon build

``eplansys_rmf_bridge`` finds PlanSys2 with ``QUIET`` and builds its library
without it, so the RMF components compile and pass their tests on a machine
with Open-RMF and without ``eplansys``. The performers require both.
``rmf_demos`` is not released for Humble; the demo requires a source build of
its ``humble`` branch.

Node parameters
---------------

.. list-table:: Parameters of ``rmf_action_node``.
   :header-rows: 1
   :widths: 26 26 48

   * - Parameter
     - Default
     - Meaning
   * - ``task_map``
     - ``""``
     - Path to the task map (:doc:`task_map`).
   * - ``websocket_port``
     - ``7879``
     - Port the fleet adapter connects to.
   * - ``outcome_prefix``
     - ``eplansys.outcome=``
     - Prefix identifying an outcome in a detail or log value.
   * - ``task_timeout``
     - ``120.0``
     - Seconds after which a submitted task is abandoned.

Task phases
-----------

``RmfTaskClient::status`` reports one of five phases for the request id
returned on submission. The RMF task id is known only once the response
arrives, so the request id is the handle.

.. list-table:: Task phases.
   :header-rows: 1
   :widths: 22 78

   * - Phase
     - Meaning
   * - ``Pending``
     - Submitted; no ``ApiResponse`` yet.
   * - ``Rejected``
     - Refused by RMF; reason in ``error``.
   * - ``Running``
     - Accepted and in progress.
   * - ``Succeeded``
     - Reached ``completed``.
   * - ``Failed``
     - Reached failed, canceled or killed.

Submission is deferred until the robot has appeared on ``fleet_states``. A
performer can receive its first action within a second of start-up, before the
fleet adapter is running; a ``robot_task_request`` published then is lost
without error, as only the named robot's ``TaskManager`` handles it and the
dispatcher ignores it. A robot's first ``fleet_states`` message is the earliest
evidence that its adapter is running and discovered.

Tests
-----

Unit tests cover the task map. A smoke test drives ``state_probe`` as a fleet
adapter would, including the bare ``Hello`` frame, and checks that an outcome
token is reported.

.. code-block:: bash

   colcon test --packages-select eplansys_rmf_bridge
   colcon test-result --verbose
   source install/setup.bash
   .github/workflows/smoke_test.sh

Code style
----------

The code follows the ament defaults enforced by ``ament_uncrustify`` (K&R
braces, ``const auto & x``). Open-RMF uses Allman style, so code adapted from
``rmf_ros2`` must be reformatted. Copyright notices use ``//`` line comments;
``ament_copyright`` does not recognise block comments.

.. code-block:: bash

   ament_uncrustify eplansys_rmf_bridge eplansys_rmf_probe
   ament_copyright eplansys_rmf_bridge eplansys_rmf_probe

``ament_uncrustify --reformat`` fixes formatting in place. CI runs both
linters, the build, the unit tests and the smoke test in
``ros:humble-ros-base-jammy``. That image does not contain ``eplansys``, so CI
covers the RMF components and none of the performers.
