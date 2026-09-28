^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package plansys2_epistemic_executor
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.3.0 (2026-09-28)
------------------
* The tracked model is repaired only when the task asks for repair, as the
  planner's is, and is never refused.
* Independent runs of a policy can be dispatched together (parallel_groups).
* schedule_policy moves an independent action above a branch point when every
  branch begins with it, and groups sensing actions that run side by side.

0.2.0 (2026-09-17)
------------------
* Execution of epistemic policies as behavior trees
* EPDDL as the input, instead of a grounded task made by hand
* The epistemic state as a node of the system
* A goal and an announcement, so the epistemic state answers to a person
* End-to-end tests for the epistemic execution path
* Destroyed the solvers before unloading the libraries that define them
* The epistemic half becomes something you can run
* Leave the test processes through _exit, before static destruction
* Turn the linters on for the epistemic packages
