ament_add_gtest(policy_test policy_test.cpp TIMEOUT 120)
target_link_libraries(policy_test ${PROJECT_NAME}_policy)

ament_add_gtest(policy_bt_test policy_bt_test.cpp TIMEOUT 120)
target_link_libraries(policy_bt_test ${PROJECT_NAME}_policy behaviortree_cpp::behaviortree_cpp)

# Loads the node library the way the executor does, by path, and checks a
# rendered tree instantiates against the real nodes rather than stubs.
ament_add_gtest(bt_plugin_test bt_plugin_test.cpp TIMEOUT 120)
target_compile_definitions(bt_plugin_test PRIVATE
  EPISTEMIC_BT_NODES_LIBRARY="$<TARGET_FILE:plansys2_epistemic_bt_nodes>")
target_link_libraries(bt_plugin_test ${PROJECT_NAME} behaviortree_cpp::behaviortree_cpp)
add_dependencies(bt_plugin_test plansys2_epistemic_bt_nodes)

# The state node over its real services. The grounded tasks live with the
# planner that produces them; this test reaches across for one, at build time.
ament_add_gtest(epistemic_state_node_test epistemic_state_node_test.cpp TIMEOUT 120)
target_compile_definitions(epistemic_state_node_test PRIVATE
  EPISTEMIC_TASK_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../../plansys2_epistemic_planner/test/tasks")
target_link_libraries(epistemic_state_node_test ${PROJECT_NAME})
