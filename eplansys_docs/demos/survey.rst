The site survey
===============

Three robots, and a mission whose difficulty is entirely in who is allowed to
know what.

A site may be contaminated. Only a robot standing on the site can tell, and a
robot watching another one scan learns that it scanned, not what it found. So
far this is :doc:`corridor` with a third robot.

What makes it a different problem is that third robot. ``observer`` is a machine
the team does not control --- a contractor's drone, a public telemetry feed ---
and the mission forbids it learning the finding. That single negative goal is
what the domain is built around. It rules out the open channel, which is
otherwise the obvious way for the team to inform itself, and forces the planner
to choose a channel by who can hear it.

Unlike the corridor, this mission is stated in EPDDL and ground by ``plank`` at
start up, which is the ordinary way to pose an epistemic problem. It needs
``plank`` built and on ``PATH``.

Running it
----------

.. code-block:: bash

   ros2 launch eplansys_demo survey_launch.py
   ros2 launch eplansys_demo survey_launch.py site:=clean

The goal
--------

.. code-block:: lisp

   (:goal
     (and
       ([Kw. scout] (contaminated))
       ([Kw. relay] (contaminated))
       (<Kw. observer> (contaminated))))

Three conjuncts, and each rules out a different shortcut. Somebody has to go and
look. The finding has to travel. And it cannot travel over the open channel.

The third conjunct is the one with no classical counterpart at all. It is not a
fact to be achieved but a fact to be *avoided being known*, and no arrangement
of PDDL predicates expresses it: the observer's ignorance is not a property of
the world, and nothing the robots do to the world can secure it.

The channels
------------

Every way of speaking in this domain has the same effect on the world --- none
--- and differs only in its audience. That is what a classical planner cannot
tell apart, and what the epistemic layer is for.

.. list-table::
   :header-rows: 1
   :widths: 26 30 44

   * - Action
     - EPDDL action type
     - Who comes to know
   * - ``scan``
     - ``semi-private-sensing``
     - the scanner learns the finding; everyone else learns that a scan
       happened
   * - ``broadcast``
     - ``public-announcement``
     - everyone, including the observer
   * - ``relay``
     - ``private-announcement``
     - the speaker and the named listener; everyone else is oblivious, and
       does not learn that anything was said

What the planner does with them
-------------------------------

.. code-block:: text

   [aostar] Solution found at depth 3  Expanded=40  Generated=57
   [planner] [epistemic] policy with 4 nodes, branching

   applied goto-site_relay:                    2 worlds, 2 designated
   applied scan_relay -> e-scan-dirty:         2 worlds, 1 designated
   applied relay-dirty_relay_scout:            3 worlds, 1 designated
   [goal] (and (Kw scout contaminated)
               (Kw relay contaminated)
               (not (Kw observer contaminated))) holds

Two things in that trace are worth stopping on.

**The planner declined to broadcast.** Nothing told it not to. It was given a
public announcement that would inform the whole team in one step, and it chose a
private one instead, because the public one would have made the third conjunct
false. Which channel to use is a consequence of the goal, worked out rather than
configured.

**The model grew.** Two worlds before the relay, three after. A private
announcement adds a distinction rather than removing one: there is now a world
in which the message was sent and a world the observer still believes in, where
it was not. Sensing narrows a model and private speech widens it, and both are
the same product update.

What removing a conjunct does
-----------------------------

The goal is three constraints, and the mission changes shape when any of them
goes. Measured, by removing each and re-planning:

.. list-table::
   :header-rows: 1
   :widths: 40 60

   * - Goal
     - What the planner does
   * - all three
     - a team member scans, then uses the private link. Three actions, 40
       expansions
   * - without ``(<Kw. observer> ...)``
     - sends ``observer`` itself to scan, then broadcasts. Three actions, 32
       expansions
   * - ``[relay]`` weakened to the second-order
       ``([relay] ([Kw. scout] (contaminated)))``
     - two actions: go and scan, and nothing else

The first two rows are the same length, which is the opposite of what one
expects. Secrecy does not cost the mission a step; it makes a whole class of
solutions inadmissible, and the extra work shows up in the search rather than in
the plan.

The third row is the one worth dwelling on. Asking that ``relay`` know *that
scout knows* is already satisfied by the scan, because semi-private sensing lets
the others see that a scan happened. It is a real epistemic goal and the mission
gets it for nothing --- which is a good illustration of why second-order goals
have to be read carefully before they are relied on.
