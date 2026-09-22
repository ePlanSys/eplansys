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
one node at a time and the mission takes about twenty-four seconds: eighteen of
declared duration and the rest in dispatch.

With it on, the builder looks for runs of the policy whose actions are
independent --- classically, by asking the domain whether either one's effects
touch what the other requires, and epistemically, by requiring that the agents
they name be disjoint --- and renders each run as one ``Parallel``. In the
policy this mission produces, the report from the north site and the drive to
the south one are such a run, and the mission takes about twenty-one and a half
seconds.

The saving is the shorter of the two overlapped actions, and it is smaller than
the halves of the mission would allow. A policy orders its nodes, and the two
halves of this one are interleaved rather than adjacent: the drive south sits
below the branch on what the north scan found, so only the pair that happens to
be consecutive can be grouped. Lifting an independent action across a branch
point, so that both halves run from the start, is what the pass does not yet do.

.. code-block:: text

   dispatching together: (relay_north north relay), (goto_south south)

is what it says when it finds one.
