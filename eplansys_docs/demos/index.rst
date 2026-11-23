Demos
=====

Missions in ``eplansys_demo``, each started by one launch command. None
requires a simulator or hardware; the performers are stand-ins that wait for
their declared duration and report a configured outcome. All three missions
run on ``MissionSupervisor`` (:ref:`arch-supervisor`) and require ``plank`` on
``PATH``.

.. list-table:: Demo missions.
   :header-rows: 1
   :widths: 22 12 66

   * - Mission
     - Agents
     - Epistemic feature
   * - :doc:`corridor`
     - 2
     - Branching on a sensing outcome; knowing-whether goal
   * - :doc:`survey`
     - 3
     - Negative goal (an agent must not learn the finding); choice of channel
       by audience
   * - :doc:`two_sites`
     - 4
     - Two independent subpolicies; parallel dispatch

.. toctree::
   :maxdepth: 1

   corridor
   survey
   two_sites
