Installation
============

ePlanSys is built as a ROS 2 workspace with ``colcon``. The whole repository,
including the classical PlanSys2 packages, builds on ROS 2 Rolling; the
epistemic packages that do not depend on ``plansys2_executor`` also build on
Humble. Both configurations are the ones exercised in continuous integration
by ``.github/workflows/rolling.yaml`` and
``.github/workflows/epistemic-humble.yaml``.

Supported distributions
-----------------------

.. list-table::
   :header-rows: 1
   :widths: 20 30 50

   * - Distribution
     - Workflow
     - Packages built
   * - Rolling
     - ``rolling.yaml``
     - All packages in the repository, including
       ``plansys2_epistemic_bt_builder``.
   * - Humble
     - ``epistemic-humble.yaml``
     - ``plansys2_epddl_grounder``, ``plansys2_epistemic_planner``,
       ``plansys2_epistemic_msgs``, ``plansys2_epistemic_executor``,
       ``plansys2_epistemic_perception``, ``plansys2_aletheia_plan_solver``
       and ``plansys2_tui_cli``. ``plansys2_epistemic_bt_builder`` is absent
       because it is the one epistemic package that depends on
       ``plansys2_executor``.

Two differences between the distributions are what the list above turns on,
and both are asked about as questions about the API rather than about the
distribution's name, in ``plansys2_core/Compat.hpp``:

``plansys2::SpinExecutor``
   ``rclcpp::experimental::executors::EventsExecutor`` where the header
   exists, which is Iron and later, and
   ``rclcpp::executors::SingleThreadedExecutor`` on Humble, where it does not.
   Bringing a system up is a burst of service calls and transition
   notifications, which is the traffic a wait-set executor handles worst, so
   the events executor is preferred wherever it is available.

``plansys2::service_qos()``
   An ``rclcpp::QoS`` from rclcpp 17 onwards, and the ``rmw`` profile that
   Humble's ``create_service`` still takes below it. The two do not convert,
   so the call site cannot be written once for both without this.

A distribution that changes back, or a backport, is therefore handled without
editing that header.

Dependencies
------------

Beyond a ROS 2 installation, the epistemic packages need:

* ``nlohmann_json`` 3.10 or newer, required by ``plansys2_epistemic_planner``
  and re-exported on its interface.
* ``behaviortree_cpp``, required by ``plansys2_epistemic_executor``.
* ``pluginlib``, ``rclcpp``, ``rclcpp_lifecycle`` and ``std_msgs``.

``plansys2_tui_cli`` is pure Python and needs ``rclpy``, ``ros2cli``,
``python3-rich``, ``python3-platformdirs`` and ``python3-typing-extensions``.
It vendors the Textual library it draws its dashboard with, so nothing has to
be installed with pip; the two ``python3-`` packages are what the vendored
copy expects to find in the distribution, and are backfilled where Humble's
are older than it wants.

``plansys2_epddl_grounder`` needs the ``plank`` EPDDL toolkit at run time,
which is the third source dependency in ``dependency_repos.repos``. It is a
plain CMake project rather than a ROS package, so colcon builds it as one and
installs its binary into ``<prefix>/bin``, which puts ``plank`` on PATH once
the workspace is sourced; building it requires ``libboost-filesystem-dev``.
Nothing links against it, so a workspace built without it still builds and
still passes its tests. Only the EPDDL front end is unavailable, and the
grounder says so rather than failing obscurely.

The classical PlanSys2 packages additionally need two source dependencies,
listed in the same file: ``popf`` and ``cascade_lifecycle``. Building ``popf``
requires ``libfl-dev``.

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

``eplansys`` is a metapackage that depends on PlanSys2 and on every epistemic
package, so ``--packages-up-to eplansys`` builds the whole system and
installing it as a binary package pulls the set in. It contains no code of its
own.

To build only the epistemic chain, which needs neither ``popf`` nor
``cascade_lifecycle``:

.. code-block:: bash

   colcon build --packages-up-to plansys2_epistemic_executor plank

``plank`` is named explicitly because nothing declares a build dependency on
it: ``plansys2_epddl_grounder`` runs it as a subprocess and finds it on PATH,
so ``--packages-up-to`` would never reach it.

Note that ``plansys2_epistemic_bt_builder`` is deliberately excluded from that
chain: it is the only epistemic package that depends on ``plansys2_executor``,
which is what keeps the rest buildable against a released distribution.

Language standard
-----------------

The epistemic packages set ``CMAKE_CXX_STANDARD 23`` with extensions
disabled, so a compiler with C++23 support is required. The classical
PlanSys2 packages are unaffected.

Running the tests
-----------------

.. code-block:: bash

   colcon test --packages-select plansys2_epddl_grounder plansys2_epistemic_planner \
     plansys2_epistemic_executor plansys2_epistemic_perception plansys2_tui_cli
   colcon test-result --verbose

The tests that run without a live ROS graph cover the EPDDL grounder, the task
parser, the search strategies, the action mapping, the policy plan, the policy
reading, and the
rendering of a policy into a behavior tree that the real BehaviorTree.CPP
factory accepts. ``plansys2_epistemic_perception`` additionally brings up a
real ``epistemic_state`` node and checks that what it reads off a grid arrives
there as the outcome or announcement it was configured to send, rather than
against a stand-in for that node. ``plansys2_tui_cli``'s tests are the three
ament linters and a check that every declared entry point imports, which is
what catches a verb that would only fail when a user typed it. The grounder's own tests run against a stand-in for plank, so
they pass whether or not it is built; the one test that grounds the packaged
example through the real binary skips when it is absent. The behavior tree
nodes ticking against a running
``epistemic_state`` node, and the builder inside a running executor, are not
covered by these tests; they require a full PlanSys2 stack and are exercised
on Rolling only.
