^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package plansys2_aletheia_plan_solver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.0 (2026-09-17)
------------------
* One Aletheia behind both plugins: the binary this package runs and the
  library plansys2_epistemic_planner links are built from one pinned checkout,
  and a test holds the two to the same policy on the fleet tasks.
* Aletheia plan solver as an external process
* Fleet scenarios, so the worked example is a robot
* Leave the test processes through _exit, before static destruction
