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
     - ``plansys2_epistemic_planner``, ``plansys2_epistemic_msgs`` and
       ``plansys2_epistemic_executor`` only. The remaining packages use
       ``rclcpp::experimental::executors``, which Humble does not provide.

Dependencies
------------

Beyond a ROS 2 installation, the epistemic packages need:

* ``nlohmann_json`` 3.10 or newer, required by ``plansys2_epistemic_planner``
  and re-exported on its interface.
* ``behaviortree_cpp``, required by ``plansys2_epistemic_executor``.
* ``pluginlib``, ``rclcpp``, ``rclcpp_lifecycle`` and ``std_msgs``.

The classical PlanSys2 packages additionally need two source dependencies,
listed in ``dependency_repos.repos``: ``popf`` and ``cascade_lifecycle``.
Building ``popf`` requires ``libfl-dev``.

Building from source
--------------------

.. code-block:: bash

   mkdir -p ~/eplansys_ws/src
   cd ~/eplansys_ws/src
   git clone https://github.com/ePlanSys/eplansys.git

   cd ~/eplansys_ws
   vcs import src < src/eplansys/dependency_repos.repos

   sudo apt-get update && sudo apt-get install -y libfl-dev
   rosdep install --from-paths src --ignore-src -r -y

   colcon build --symlink-install
   source install/setup.bash

To build only the epistemic chain, which needs neither ``popf`` nor
``cascade_lifecycle``:

.. code-block:: bash

   colcon build --packages-up-to plansys2_epistemic_executor

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

   colcon test --packages-select plansys2_epistemic_planner plansys2_epistemic_executor
   colcon test-result --verbose

The tests that run without a live ROS graph cover the task parser, the search
strategies, the action mapping, the policy plan, the policy reading, and the
rendering of a policy into a behavior tree that the real BehaviorTree.CPP
factory accepts. The behavior tree nodes ticking against a running
``epistemic_state`` node, and the builder inside a running executor, are not
covered by these tests; they require a full PlanSys2 stack and are exercised
on Rolling only.
