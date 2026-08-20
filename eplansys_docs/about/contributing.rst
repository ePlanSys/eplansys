Contributing
============

Contributions are welcome. This page summarises what a patch has to satisfy;
the authoritative texts are ``CONTRIBUTING.md`` and ``CODE_OF_CONDUCT.md`` at
the root of the repository.

Licensing
---------

Contributions are made under the predominant license of the package they touch.
Entirely new packages are made available under the `Apache License, Version 2.0
<https://www.apache.org/licenses/LICENSE-2.0>`_. See :doc:`license` for how
that applies to this repository as a whole.

Developer Certificate of Origin
-------------------------------

Every commit must carry a ``Signed-off-by`` line, which is the contributor's
attestation to the `Developer Certificate of Origin
<https://developercertificate.org/>`_:

.. code-block:: text

   Signed-off-by: Sofforus Jones <sjones@gmail.com>

Adding ``-s`` or ``--signoff`` to the commit command inserts it. A commit that
is missing it can be amended with ``git commit --amend -s``, which requires a
force push if the branch was already pushed. The name and email on the sign-off
must match the account submitting the pull request.

Branches
--------

``rolling`` is the main development branch and the one pull requests target.
The per-distribution branches, ``humble-devel`` and the rest, exist for their
own distributions and have their own workflows.

Continuous integration
----------------------

A pull request against ``rolling`` runs two build-and-test workflows:

``rolling.yaml``
   Builds and tests every package in the repository in the ``ros:rolling``
   container, and uploads coverage to Codecov.

``epistemic-humble.yaml``
   Builds and tests ``plansys2_epistemic_planner``,
   ``plansys2_epistemic_msgs`` and ``plansys2_epistemic_executor`` in the
   ``ros:humble`` container. Adding a dependency on ``plansys2_executor`` to
   any of those three would break this workflow, which is the point of the
   package split described in :doc:`../concepts/architecture`.

A change under ``eplansys_docs/`` additionally runs ``docs.yaml``, which builds
this site with warnings treated as errors. A build warning is a failed build,
so a new page has to be reachable from a toctree and every cross-reference it
makes has to resolve.

Style
-----

The epistemic packages run ``ament_lint_auto`` in their test targets, so a
patch has to satisfy the ament linters for the package it touches. They are
built with ``CMAKE_CXX_STANDARD 23`` and extensions disabled.
