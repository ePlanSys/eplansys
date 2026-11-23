^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package plansys2_epistemic_executor
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.4.1 (2026-11-22)
------------------
* Released with the PlanSys2 packages at 3.1.0, so that apt upgrades them
  together. No source changes.

0.4.0 (2026-11-22)
------------------
* temporal_policy_to_bt renders a temporal policy as a tree dispatched by
  events, with the WaitForEnds, CheckGiven and MarkDone nodes. A branch the
  world did not take is skipped item by item.
* Policy::validate checks a temporal plan's starts_after and given lists.

0.3.5 (2026-11-22)
------------------
* The README describes MissionSupervisor and how a contradicted policy fails.

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
