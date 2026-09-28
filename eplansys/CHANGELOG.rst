^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package eplansys
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.3.0 (2026-09-28)
------------------
* del-planner (formerly Aletheia) is pinned at 314bc29, with --consistent-beliefs, and
  its URL now names it del-planner.
* A Docker image on ROS 2 Humble runs the two-site survey from a clean clone.
* The survey mission asks for a policy again when an action fails, from the
  epistemic state it reached (eplansys_demo).

0.2.0 (2026-09-17)
------------------
* Aletheia and plank are pinned to commits in dependency_repos.repos.
* Install the epistemic half alongside PlanSys2 as one unit
