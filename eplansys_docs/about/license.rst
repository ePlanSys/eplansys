License
=======

ePlanSys is distributed under the `Apache License, Version 2.0
<https://www.apache.org/licenses/LICENSE-2.0>`_. The full text is in
``LICENSE`` at the repository root and the attribution notice in ``NOTICE``.

Relationship to PlanSys2
------------------------

ePlanSys is a derivative work of `PlanSys2
<https://github.com/PlanSys2/ros2_planning_system>`_
:cite:p:`martin2021plansys2`, developed by Francisco Martín Rico, Jonatan
Ginés Clavero, Francisco J. Rodríguez Lera and Vicente Matellán Olivera and
licensed under the Apache License, Version 2.0.

ePlanSys is not endorsed by, affiliated with, or supported by the PlanSys2
project or its authors. Issues and pull requests concerning this repository
belong here.

Provenance
----------

.. list-table:: Origin of the packages.
   :header-rows: 1
   :widths: 30 70

   * - Packages
     - Origin
   * - Non-epistemic ``plansys2_*``
     - PlanSys2, with upstream commit history preserved (``git log``
       identifies the origin of each file). All are modified, in four
       categories: extension points (principally ``plansys2_msgs`` and
       ``plansys2_executor``), bring-up and lifecycle management of the
       epistemic nodes, Humble/Rolling compatibility shims, and defect
       fixes. ``plansys2_tui_cli`` (the ``epistemic`` verb and a Textual
       dashboard) and ``plansys2_tests`` (epistemic integration tests) are
       modified more extensively.
   * - Epistemic packages, ``eplansys``, ``eplansys_demo``
     - Original contributions, developed as part of a terminal work at the
       Instituto Politécnico Nacional, Mexico. The ``plansys2_`` prefix of
       several of them names the extension point they attach to; it does
       not indicate authorship. ``NOTICE`` lists them.
   * - Planning core of ``plansys2_epistemic_planner``
     - Aletheia (`del-planner <https://github.com/HanielUlises/del-planner>`_)
       :cite:p:`aletheia`, by the same author, Apache License 2.0: state
       representation, product update, bisimulation contraction, formulas and
       model checking, heuristics, search strategies, selection policy, parser
       and validator. Built from source through ``dependency_repos.repos`` and
       linked as a library; not copied into this repository. The plugin,
       policy and belief serialisation, action mapping and formula front end
       in ``plansys2_epistemic_planner`` are original to this repository.
   * - Three files in ``plansys2_epddl_grounder``
     - Reproduced verbatim from other projects; used at run time and not
       compiled. ``NOTICE`` names them and their licences.

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

The link-preview image contains the ROS logo from `ros-infrastructure/artwork
<https://github.com/ros-infrastructure/artwork>`_, licensed under CC BY-NC 4.0
and used unmodified under the ROS trademark policy. ROS is a trademark of Open
Robotics.
