ament_add_gtest(policy_test policy_test.cpp TIMEOUT 120)
target_link_libraries(policy_test ${PROJECT_NAME}_policy)

ament_add_gtest(policy_bt_test policy_bt_test.cpp TIMEOUT 120)
target_link_libraries(policy_bt_test ${PROJECT_NAME}_policy behaviortree_cpp::behaviortree_cpp)

# The grounded tasks live with the planner that produces them; only this test
# reaches across, and only at build time.
ament_add_gtest(formula_text_test formula_text_test.cpp TIMEOUT 120)
target_compile_definitions(formula_text_test PRIVATE
  EPISTEMIC_TASK_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../../plansys2_epistemic_planner/test/tasks")
target_link_libraries(formula_text_test ${PROJECT_NAME})

# Loads the node library the way the executor does, by path, and checks a
# rendered tree instantiates against the real nodes rather than stubs.
ament_add_gtest(bt_plugin_test bt_plugin_test.cpp TIMEOUT 120)
target_compile_definitions(bt_plugin_test PRIVATE
  EPISTEMIC_BT_NODES_LIBRARY="$<TARGET_FILE:plansys2_epistemic_bt_nodes>")
target_link_libraries(bt_plugin_test ${PROJECT_NAME} behaviortree_cpp::behaviortree_cpp)
add_dependencies(bt_plugin_test plansys2_epistemic_bt_nodes)
