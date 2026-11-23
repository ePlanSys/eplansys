Installation
============

ePlanSys is installed from the Debian packages attached to each release
(Humble only) or built from source as a ROS 2 workspace with ``colcon``
(Humble and Rolling).

Release packages
----------------

On Ubuntu 22.04 with ROS 2 Humble, each GitHub release is an apt repository:

.. code-block:: bash

   echo "deb [trusted=yes] https://github.com/ePlanSys/eplansys/releases/download/v0.3.0 ./" |
     sudo tee /etc/apt/sources.list.d/eplansys.list
   sudo apt update
   sudo apt install ros-humble-eplansys ros-humble-eplansys-demo

``ros-humble-eplansys`` installs the epistemic packages, the planner
(``ros-humble-aletheia``, built from del-planner) and the grounder
(``ros-humble-plank``) at the revisions the release pins. The PlanSys2
packages are this repository's fork, released under the upstream names at
version 3.0.0, and take precedence over the upstream 2.0.x packages.
``.github/workflows/debs.yaml`` builds them with ``packaging/build-debs.sh``
when a release is tagged.

Supported distributions
-----------------------

.. list-table:: Continuous integration and packaging.
   :header-rows: 1
   :widths: 16 26 58

   * - Distribution
     - Workflow
     - Scope
   * - Rolling
     - ``rolling.yaml``
     - Build and test of every package; coverage uploaded to Codecov.
   * - Humble
     - ``epistemic-humble.yaml``
     - Build and test of the epistemic packages, ``plansys2_executor``,
       ``plansys2_epistemic_bt_builder``, ``plansys2_tests`` and
       ``eplansys_demo``. ``uncrustify`` is excluded on this job: Humble's
       version formats two lines of ``plansys2_executor`` differently from
       Rolling's, and the Rolling job is the style gate.
   * - Humble
     - ``debs.yaml``
     - Debian packages of every released package, on a release tag.

The source differences between the two distributions are isolated in
``plansys2_core/Compat.hpp``, which tests for API features and never for
the distribution name:

``plansys2::SpinExecutor``
   ``rclcpp::experimental::executors::EventsExecutor`` where the header exists
   (Iron and later), otherwise ``rclcpp::executors::SingleThreadedExecutor``.
   Bring-up consists of bursts of service calls and transition notifications,
   for which the events executor performs better than a wait-set executor.

``plansys2::service_qos()``
   An ``rclcpp::QoS`` from rclcpp 17 onwards, and the ``rmw`` QoS profile that
   Humble's ``create_service`` takes otherwise. The two types do not convert.

Dependencies
------------

.. list-table:: Dependencies beyond a ROS 2 installation.
   :header-rows: 1
   :widths: 30 25 45

   * - Dependency
     - Required by
     - Notes
   * - ``nlohmann_json`` ≥ 3.10
     - ``plansys2_epistemic_planner``
     - Re-exported on the package's interface.
   * - ``behaviortree_cpp``
     - ``plansys2_epistemic_executor``
     -
   * - ``pluginlib``, ``rclcpp``, ``rclcpp_lifecycle``, ``std_msgs``
     - epistemic packages
     -
   * - ``rclpy``, ``ros2cli``, ``python3-rich``, ``python3-platformdirs``,
       ``python3-typing-extensions``
     - ``plansys2_tui_cli``
     - Textual is vendored; the two ``python3-`` packages are backfilled
       where Humble's are older than it expects.
   * - Aletheia (`del-planner <https://github.com/HanielUlises/del-planner>`_)
     - ``plansys2_epistemic_planner``, ``plansys2_aletheia_plan_solver``
     - Source dependency, pinned. Built as a library (linked) and as the
       ``epistemic_planner`` binary (run by the subprocess plugin).
   * - `plank <https://github.com/HanielUlises/plank>`_
     - ``plansys2_epddl_grounder`` (run time)
     - Source dependency, pinned. Plain CMake project; requires
       ``libboost-filesystem-dev``.
   * - ``popf``, ``cascade_lifecycle``
     - classical PlanSys2 packages
     - Source dependencies; ``popf`` requires ``libfl-dev``.

All source dependencies are listed with their pinned revisions in
``dependency_repos.repos``.

The ``epistemic_planner`` binary is compiled with ``-march=native``; pass
``--cmake-args -DALETHEIA_NATIVE=OFF`` when the build machine differs from the
target. The library is never compiled with it.

plank is installed into ``<prefix>/bin`` and found on ``PATH``. No package
links against it, so a workspace without it builds and passes its tests; only
the EPDDL front end is unavailable, and it reports this explicitly.

Building from source
--------------------

.. code-block:: bash

   mkdir -p ~/eplansys_ws/src
   cd ~/eplansys_ws/src
   git clone https://github.com/ePlanSys/eplansys.git

   cd ~/eplansys_ws
   vcs import src < src/eplansys/dependency_repos.repos

   sudo apt-get update
   sudo apt-get install -y libfl-dev libboost-filesystem-dev
   rosdep install --from-paths src --ignore-src -r -y

   colcon build --symlink-install
   source install/setup.bash

``eplansys`` is a metapackage depending on PlanSys2 and every epistemic
package; ``--packages-up-to eplansys`` builds the complete system.

The epistemic chain alone, without ``popf`` and ``cascade_lifecycle``:

.. code-block:: bash

   colcon build --packages-up-to plansys2_epistemic_executor plank

``plank`` is named explicitly because no package declares a build dependency on
it. ``plansys2_epistemic_bt_builder`` is not in this chain; it is the only
epistemic package that depends on ``plansys2_executor``.

The epistemic packages set ``CMAKE_CXX_STANDARD 23`` with extensions disabled
and require a C++23 compiler. The classical packages are unaffected.

Tests
-----

.. code-block:: bash

   colcon test --packages-select plansys2_epddl_grounder plansys2_epistemic_planner \
     plansys2_epistemic_executor plansys2_epistemic_perception plansys2_tui_cli
   colcon test-result --verbose

.. list-table:: Coverage of the package tests.
   :header-rows: 1
   :widths: 32 68

   * - Package
     - Tests
   * - ``plansys2_epddl_grounder``
     - Grounding against a stand-in for plank; one test grounds the packaged
       example with the real binary and is skipped when it is absent.
   * - ``plansys2_epistemic_planner``
     - Task parser, search strategies, action mapping, policy plan.
   * - ``plansys2_epistemic_executor``
     - Policy reading and rendering into a tree accepted by the
       BehaviorTree.CPP factory; the ``epistemic_state`` node.
   * - ``plansys2_epistemic_perception``
     - Region classification, against a live ``epistemic_state`` node.
   * - ``plansys2_tui_cli``
     - ament linters; import of every declared entry point.

The behavior tree nodes against a live ``epistemic_state``, and the builder
inside a running executor, require the full PlanSys2 stack. They are covered
by ``plansys2_tests`` (tests 5 to 7) on Rolling and Humble
(:doc:`../reports/epistemic_end_to_end`).
