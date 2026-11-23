
ament_add_gtest(test_8 test_8.cpp TIMEOUT 300)
# The grounded task and the action mapping live with the planner that produces
# them; only this test reaches across, and only at build time.
target_compile_definitions(test_8 PRIVATE
  EPISTEMIC_TASK_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../../plansys2_epistemic_planner/test/tasks"
  EPISTEMIC_MAPPING_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../../plansys2_epistemic_planner/examples/mappings"
  # Test 5's PDDL; the node library the executor loads by path, as it would from a parameter
  # file. Nothing else is needed: this mission runs on the packaged action
  # template, and its observation comes from the performer.
  EPISTEMIC_BT_NODES_LIBRARY="$<TARGET_FILE:plansys2_epistemic_executor::plansys2_epistemic_bt_nodes>"
)
target_link_libraries(test_8
  ${PROJECT_NAME}
  ament_index_cpp::ament_index_cpp
  plansys2_domain_expert::plansys2_domain_expert
  plansys2_problem_expert::plansys2_problem_expert
  plansys2_planner::plansys2_planner
  plansys2_epistemic_executor::plansys2_epistemic_executor
)
