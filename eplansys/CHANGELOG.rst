^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package eplansys
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.3.5 (2026-11-22)
------------------
* MissionSupervisor (plansys2_executor) plans again when a mission fails, from
  the problem and the belief the failed attempt left, and gives up after
  mission.max_attempts_without_progress attempts in which no action completed.
  The corridor, survey and two-site missions all run on it (eplansys_demo).
* The executor's replan guard no longer counts the synthetic (INIT) entry as
  progress, so it allows exactly max_replans_without_progress fruitless replans.
* An action executor stops requesting performers once the behavior tree has
  left its node, and ExecuteAction::halt() cancels the action. A performer lets
  go of an answer that is neither confirmed nor rejected within two seconds
  (C++ and Python clients). WaitAtStartReq takes an optional
  at_start_req_timeout; the default of zero keeps the unbounded wait.
* The demo performers exit without running static destructors (eplansys_demo).
* test_7 runs the corridor mission against a world the policy did not plan for
  (plansys2_tests).
* CI builds and tests the executor side, the epistemic BT builder, the tests
  and the demos on Humble; the workflows for upstream's branches are gone.

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
