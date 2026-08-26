^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package plansys2_epistemic_planner
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Adding planner
* Adding planner
* Adding planner
* Adding planner
* Adding planner
* Re-sync vendored Aletheia soruces
* Epistemic Plan Solver plugin
* Tests for the epistemic planner
* Translated grounded actions into PlanSys2 actions equivalents Aletheia's
  action names and PlanSys2's are two vocabularies, not two spellings of
  one. plank grounds an action into a single token (pickup-A-hold_r2), while
  the executor splits (pickup r2 A) into a name and parameters and looks the
  name up in the PDDL domain to find the BT that drives the hardware.
  Emitting the grounded name matched nothing there, so a plan reached the
  executor undispatchable and the robot did not move.
* Serialization of AO* policies instead of flattening them
* EPDDL as the input, instead of a grounded task made by hand
* A goal and an announcement, so the epistemic state answers to a person
* Fleet scenarios, so the worked example is a robot
* End-to-end tests for the epistemic execution path
* The epistemic half becomes something you can run
* Documentation of the fleet scenarios
* Leave the test processes through _exit, before static destruction
* Notice on Aletheia
* Turn the linters on for the epistemic packages
