The two-site survey
===================

:doc:`survey` with two independently contaminated sites, ``north`` and
``south``. The scouts ``north`` and ``south`` each carry the only instrument
that reads their own site; ``relay`` must learn both findings and ``observer``
must learn neither. The goal has six conjuncts, the survey's three per site.

The two halves of the policy share no atoms, agents or locations, and no action
of one occurs in a precondition of the other. Dispatched one node at a time,
the policy takes the sum of both halves' durations; with parallel dispatch the
halves overlap.

The EPDDL sources are grounded by ``plank`` at start-up.

Usage
-----

.. code-block:: bash

   ros2 launch eplansys_demo survey_sites_launch.py
   ros2 launch eplansys_demo survey_sites_launch.py north:=clean south:=dirty
   ros2 launch eplansys_demo survey_sites_launch.py parallel:=true

``north`` and ``south`` set the reported state of each site; each of the four
combinations executes a different branch. ``parallel:=true`` enables parallel
dispatch; the policy is the same in both modes.

Goal
----

.. code-block:: lisp

   (:goal
     (and
       ([Kw. north] (contaminated-north))
       ([Kw. relay] (contaminated-north))
       (<Kw. observer> (contaminated-north))
       ([Kw. south] (contaminated-south))
       ([Kw. relay] (contaminated-south))
       (<Kw. observer> (contaminated-south))))

Modelling notes
---------------

Instruments
   Without instruments the cheapest policy sends one robot to both sites,
   which leaves nothing to overlap. Declaring, as common knowledge, that only
   ``north`` can read the north site assigns the halves to different agents.

Fully specified instruments
   An instrument atom the initial theory leaves open is one the model is
   uncertain about: eight open instrument atoms designate 256 worlds where 4
   suffice. The problem therefore states every agent's instruments.

Parallel dispatch
-----------------

With ``parallel_dispatch`` disabled (the default), nodes are dispatched one at
a time and the mission takes about 22.5 s: 18 s of declared duration plus
dispatch overhead.

With it enabled, the builder schedules each action as early as its
dependencies allow and dispatches each run of independent actions as one
``Parallel`` group (:ref:`arch-bt`). Two actions are independent when neither
one's effects touch the other's requirements (decided from the PDDL domain) and
the agents they name are disjoint. The builder logs the groups:

.. code-block:: text

   dispatching together: (goto_north north), (goto_south south)
   dispatching together: (scan_north north), (scan_south south)

The south scan occurs under both outcomes of the north scan and depends on
neither. Once the north report is moved below it, every branch of the north
scan begins with the south scan, which is therefore hoisted above the branch
point and runs concurrently with the north scan; the outcomes then select the
continuation. The mission takes about 15 s, from 11 s of declared duration.

.. list-table:: Makespan of the two-site mission.
   :header-rows: 1
   :widths: 40 30 30

   * - Dispatch
     - Declared duration
     - Measured
   * - sequential
     - 18 s
     - ≈ 22.5 s
   * - parallel
     - 11 s
     - ≈ 15 s

Each half has 9 s of declared duration. The parallel total is 11 s because both
reports name ``relay``: actions naming a common agent are serialised, so the
south report follows the north one.
