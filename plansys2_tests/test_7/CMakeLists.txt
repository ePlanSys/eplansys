ament_add_gtest(test_7 test_7.cpp TIMEOUT 300)
# The corridor's PDDL is test 5's, installed by that test. The grounded tasks
# and the action mapping live with the planner, as for tests 5 and 6.
target_compile_definitions(test_7 PRIVATE
  EPISTEMIC_TASK_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../../plansys2_epistemic_planner/test/tasks"
  EPISTEMIC_MAPPING_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../../plansys2_epistemic_planner/examples/mappings"
  EPISTEMIC_BT_NODES_LIBRARY="$<TARGET_FILE:plansys2_epistemic_executor::plansys2_epistemic_bt_nodes>"
)
target_link_libraries(test_7
  ${PROJECT_NAME}
  ament_index_cpp::ament_index_cpp
  plansys2_domain_expert::plansys2_domain_expert
  plansys2_problem_expert::plansys2_problem_expert
  plansys2_planner::plansys2_planner
  plansys2_epistemic_executor::plansys2_epistemic_executor
)
