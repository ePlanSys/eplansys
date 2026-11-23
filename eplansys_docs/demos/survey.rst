The site survey
===============

Three agents and a site that may be contaminated. As in :doc:`corridor`, only
an agent on the site can sense its state, and sensing is semi-private. The
third agent, ``observer``, represents a party outside the team's control (a
contractor's drone, a public telemetry feed), and the goal requires that it not
learn the finding. This negative conjunct excludes the public channel and makes
the choice of channel depend on its audience.

The EPDDL sources are grounded by ``plank`` at start-up.

Usage
-----

.. code-block:: bash

   ros2 launch eplansys_demo survey_launch.py
   ros2 launch eplansys_demo survey_launch.py site:=clean

The mission node replans from the reached belief when an action fails, up to
``mission.max_attempts`` attempts (default 3 for this mission;
:ref:`arch-supervisor`).

Goal
----

.. code-block:: lisp

   (:goal
     (and
       ([Kw. scout] (contaminated))
       ([Kw. relay] (contaminated))
       (<Kw. observer> (contaminated))))

The first two conjuncts require sensing and communication. The third,
:math:`\neg\mathit{Kw}_{\mathit{observer}}\,\mathit{contaminated}`, is a
condition on an agent's ignorance. It is not a property of the world and has no
PDDL representation.

Channels
--------

The communication actions have no ontic effect; they differ only in audience
(:numref:`tab-survey-channels`).

.. _tab-survey-channels:

.. list-table:: Actions of the survey domain.
   :header-rows: 1
   :widths: 26 30 44

   * - Action
     - EPDDL action type
     - Resulting knowledge
   * - ``scan``
     - ``semi-private-sensing``
     - the scanner learns the finding; the others learn that a scan occurred
   * - ``broadcast``
     - ``public-announcement``
     - all agents, including ``observer``, learn the finding
   * - ``relay``
     - ``private-announcement``
     - the speaker and the named listener; the others are oblivious and do
       not learn that a message was sent

Result
------

.. code-block:: text

   [aostar] Solution found at depth 3  Expanded=40  Generated=57
   [planner] [epistemic] policy with 4 nodes, branching

   applied goto-site_relay:                    2 worlds, 2 designated
   applied scan_relay -> e-scan-dirty:         2 worlds, 1 designated
   applied relay-dirty_relay_scout:            3 worlds, 1 designated
   [goal] (and (Kw scout contaminated)
               (Kw relay contaminated)
               (not (Kw observer contaminated))) holds

The policy uses the private ``relay`` although ``broadcast`` would inform the
team in one action; a broadcast would falsify the third conjunct. The choice of
channel follows from the goal and is not configured.

The model grows from two worlds to three at the private announcement. A private
announcement adds a distinction: the model now contains a world in which the
message was sent and the world ``observer`` still considers actual, in which it
was not. Sensing shrinks the designated set and private announcement enlarges
the model; both are instances of the product update.

Goal ablation
-------------

.. list-table:: Effect of removing or weakening each conjunct (measured by
   re-planning).
   :header-rows: 1
   :widths: 40 60

   * - Goal
     - Policy
   * - all three conjuncts
     - a team member scans, then relays privately; 3 actions, 40 expansions
   * - without :math:`\neg\mathit{Kw}_{\mathit{observer}}`
     - ``observer`` itself scans, then broadcasts; 3 actions, 32 expansions
   * - :math:`\mathit{Kw}_{\mathit{relay}}` weakened to
       :math:`K_{\mathit{relay}}\,\mathit{Kw}_{\mathit{scout}}\,\mathit{contaminated}`
     - go and scan; 2 actions

The first two policies have the same length. The secrecy conjunct excludes a
class of solutions without lengthening the plan, and its cost appears in the
search (40 against 32 expansions).

In the third row, the second-order goal that ``relay`` know that ``scout`` knows
is satisfied by the scan alone, since semi-private sensing lets every agent
observe that a scan occurred. Second-order goals of this kind can be weaker
than their wording suggests.
