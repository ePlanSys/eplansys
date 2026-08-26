License
=======

ePlanSys is distributed under the `Apache License, Version 2.0
<https://www.apache.org/licenses/LICENSE-2.0>`_. The full text is in the
``LICENSE`` file at the root of the repository, and the attribution notice is
in ``NOTICE``.

Relationship to PlanSys2
------------------------

This project is a derivative work of `PlanSys2 (ROS2 Planning System)
<https://github.com/PlanSys2/ros2_planning_system>`_, originally developed by
Francisco Martín Rico, Jonatan Ginés Clavero, Francisco J. Rodríguez Lera and
Vicente Matellán Olivera, and licensed under the Apache License, Version 2.0.
The original work is described in F. Martín et al., "PlanSys2: A Planning
System Framework for ROS2", IROS 2021.

ePlanSys is not endorsed by, affiliated with, or supported by the PlanSys2
project or its authors. Questions, bug reports and pull requests about the code
in this repository belong here, not to the upstream project.

Provenance of the packages
--------------------------

The ``plansys2_*`` packages that are not epistemic are reproduced unmodified
from the original PlanSys2 project and remain subject to their original
copyright notices and the Apache License, Version 2.0.

The epistemic packages are original contributions, developed as part of a
terminal work at the Instituto Politécnico Nacional, Mexico, and are also
distributed under the Apache License, Version 2.0.

Within ``plansys2_epistemic_planner``, the planning core is derived from the
`Aletheia <https://github.com/HanielUlises/Aletheia>`_ epistemic planner, a
work of the same author, also under the Apache License, Version 2.0: the
Kripke state representation, the DEL product update, bisimulation contraction,
formula representation and model checking, the heuristics, the search
strategies, the selection policy, the parser and the validator. It is
incorporated in process here, rather than invoked as a binary, which is what
``plansys2_aletheia_plan_solver`` does instead. The rest of that package, the
plan solver plugin, the policy plan serialisation, the action mapping and the
formula text front end, is original to this repository.

``plansys2_epddl_grounder`` includes three files reproduced verbatim from other
projects, used at run time rather than compiled. ``NOTICE`` names them and
their licences.

Copyright
---------

.. code-block:: text

   ePlanSys - Epistemic Planning System for ROS2
   Copyright 2026 Haniel Ulises Vasquez Morales

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
