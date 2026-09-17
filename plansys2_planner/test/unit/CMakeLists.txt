ament_add_gtest(planner_test planner_test.cpp)
target_link_libraries(planner_test
  ${PROJECT_NAME}
  ament_index_cpp::ament_index_cpp
  dl
  plansys2_pddl_parser::plansys2_pddl_parser
)
target_compile_definitions(planner_test PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
# A solver that takes three seconds, loaded through pluginlib from a prefix in
# this build tree. The prefix is laid out the way an install is, so pluginlib
# finds it through the ament index, and nothing of it is installed.
set(TEST_PLUGIN_PREFIX ${CMAKE_CURRENT_BINARY_DIR}/plugin_prefix)
add_library(slow_plan_solver SHARED slow_solver/slow_plan_solver.cpp)
set_target_properties(slow_plan_solver PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY ${TEST_PLUGIN_PREFIX}/lib)
target_link_libraries(slow_plan_solver
  plansys2_core::plansys2_core
  ${plansys2_msgs_TARGETS}
  pluginlib::pluginlib
  rclcpp_lifecycle::rclcpp_lifecycle)
target_compile_definitions(slow_plan_solver PRIVATE "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

set(TEST_PLUGIN_SHARE ${TEST_PLUGIN_PREFIX}/share/plansys2_planner_test)
set(TEST_PLUGIN_INDEX ${TEST_PLUGIN_PREFIX}/share/ament_index/resource_index)
configure_file(slow_solver/package.xml ${TEST_PLUGIN_SHARE}/package.xml COPYONLY)
configure_file(slow_solver/slow_plan_solver.xml ${TEST_PLUGIN_SHARE}/slow_plan_solver.xml COPYONLY)
file(WRITE ${TEST_PLUGIN_INDEX}/packages/plansys2_planner_test "")
file(WRITE ${TEST_PLUGIN_INDEX}/plansys2_core__pluginlib__plugin/plansys2_planner_test
  "share/plansys2_planner_test/slow_plan_solver.xml")

ament_add_gtest(busy_planner_test busy_planner_test.cpp TIMEOUT 60)
target_link_libraries(busy_planner_test
  ${PROJECT_NAME}
  ament_index_cpp::ament_index_cpp
  plansys2_pddl_parser::plansys2_pddl_parser
)
target_compile_definitions(busy_planner_test PRIVATE
  "PLUGINLIB__DISABLE_BOOST_FUNCTIONS"
  TEST_PLUGIN_PREFIX="${TEST_PLUGIN_PREFIX}")
add_dependencies(busy_planner_test slow_plan_solver)
