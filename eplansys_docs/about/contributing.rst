Contributing
============

The authoritative texts are ``CONTRIBUTING.md`` and ``CODE_OF_CONDUCT.md`` at
the repository root. This page summarises the requirements for a patch.

Licensing
---------

Contributions are licensed under the predominant license of the package they
modify. New packages are licensed under the `Apache License, Version 2.0
<https://www.apache.org/licenses/LICENSE-2.0>`_ (:doc:`license`).

Developer Certificate of Origin
-------------------------------

Every commit carries a ``Signed-off-by`` line attesting to the `Developer
Certificate of Origin <https://developercertificate.org/>`_:

.. code-block:: text

   Signed-off-by: Sofforus Jones <sjones@gmail.com>

``git commit -s`` adds it; ``git commit --amend -s`` adds it to the last
commit. The name and address must match the account that submits the pull
request.

Branches
--------

``rolling`` is the development branch and the target of pull requests.
Releases are prepared on ``release/<version>`` branches, merged into
``rolling``, and tagged ``v<version>``; the tag triggers the Debian package
build.

Continuous integration
----------------------

.. list-table:: Workflows run on a pull request.
   :header-rows: 1
   :widths: 28 72

   * - Workflow
     - Scope
   * - ``rolling.yaml``
     - Build and test of every package in ``ros:rolling``; coverage uploaded
       to Codecov. Style gate (ament linters including ``uncrustify``).
   * - ``epistemic-humble.yaml``
     - Build and test in ``ros:humble`` of the epistemic packages,
       ``plansys2_executor``, ``plansys2_epistemic_bt_builder``,
       ``plansys2_tests`` and ``eplansys_demo``, excluding ``uncrustify``.
   * - ``docs.yaml``
     - Build of this site with warnings as errors, on changes under
       ``eplansys_docs/``. A new page must be reachable from a toctree and
       every cross-reference and citation must resolve.

Style
-----

The epistemic packages run ``ament_lint_auto`` in their test targets; a patch
must pass the ament linters of the packages it modifies. They are compiled as
C++23 with extensions disabled.

Documentation on this site is written as reference material: definitions,
parameters and behaviour stated in the present tense, design rationale stated
once where it constrains use, and literature cited from ``references.bib``
with ``:cite:p:``.
