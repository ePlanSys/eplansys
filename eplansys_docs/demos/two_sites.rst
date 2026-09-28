The two-site survey
===================

Two sites, two scouts, and a policy whose halves were never ordered.

This is :doc:`survey` run twice, in different hands. Two sites may be
contaminated, independently; ``north`` and ``south`` carry the instruments their
own sites read, ``relay`` is the team member both findings have to reach, and
``observer`` is the machine the mission still excludes. The six goal conjuncts
are the survey's three, once per site.

Nothing links the two halves. They name different atoms, different robots and
different places, and no action of one appears in a precondition of the other.
What the mission is for is the consequence: a policy dispatched a node at a
time runs eighteen seconds of declared duration end to end, and the halves of
it never overlap although nothing asked them not to.

Running it
----------

.. code-block:: bash

   ros2 launch eplansys_demo survey_sites_launch.py
   ros2 launch eplansys_demo survey_sites_launch.py north:=clean south:=dirty
   ros2 launch eplansys_demo survey_sites_launch.py parallel:=true

``north:`` and ``south:`` say what each site turns out to hold, and the policy
takes a different branch for each of the four combinations. ``parallel:=true``
turns on the dispatch described below. It is the same policy either way.

Like the survey, the mission is stated in EPDDL and ground by ``plank`` at start
up, so ``plank`` has to be built and on ``PATH``.

The goal
--------

.. code-block:: lisp

   (:goal
     (and
       ([Kw. north] (contaminated-north))
       ([Kw. relay] (contaminated-north))
       (<Kw. observer> (contaminated-north))
       ([Kw. south] (contaminated-south))
       ([Kw. relay] (contaminated-south))
       (<Kw. observer> (contaminated-south))))

Why the instruments
-------------------

The instruments are not decoration. Without them the cheapest policy sends one
robot to both sites, which answers the mission correctly and leaves nothing to
overlap: one robot cannot be in two places whatever the dispatch. Declaring
that only ``north`` reads the north site, as common knowledge, is what puts the
two halves in different hands.

The same goes for what the initial state leaves out. An instrument atom the
problem does not settle is one the model has to be uncertain about, and eight
such atoms designate two hundred and fifty-six worlds where four would do. Every
agent is therefore named for every instrument.

What the dispatch changes
-------------------------

With ``parallel_dispatch`` off, which is the default, the policy is dispatched
one node at a time and the mission takes about twenty-two and a half seconds:
eighteen of declared duration and the rest in dispatch.

With it on, the builder moves each action as early as the actions it depends on
allow, and dispatches each run of independent actions as one ``Parallel``. Two
actions are independent classically when neither one's effects touch what the
other requires, which the domain answers, and epistemically when the agents they
name are disjoint. It says which runs it found:

.. code-block:: text

   dispatching together: (goto_north north), (goto_south south)
   dispatching together: (scan_north north), (scan_south south)

The south scan is written under both outcomes of the north one, and depends on
neither. Every branch of the north scan begins with it once the north report
has been moved below it, so it moves above the branch point and runs beside the
north scan; the two outcomes then choose the way on, one after the other. The
mission takes about fifteen seconds, from eleven of declared duration: the
drives, the scans, and the two reports.

Each half takes nine seconds of declared duration on its own. The two together
take eleven because both reports go to ``relay``: two actions that name the
same agent run one after the other, so the south report waits for the north
one.
