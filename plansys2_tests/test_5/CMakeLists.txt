install(DIRECTORY pddl
  DESTINATION share/${PROJECT_NAME}/test_5
)

ament_add_gtest(test_5 test_5.cpp TIMEOUT 300)
# The grounded task and the action mapping live with the planner that produces
# them; only this test reaches across, and only at build time.
target_compile_definitions(test_5 PRIVATE
  EPISTEMIC_TASK_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../../plansys2_epistemic_planner/test/tasks"
  EPISTEMIC_MAPPING_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../../plansys2_epistemic_planner/examples/mappings"
  # The two node libraries the executor loads by path, as it would from a
  # parameter file. The epistemic one comes from the package that defines the
  # nodes; the other is this package's stand-in for a performer that reports
  # what it saw.
  EPISTEMIC_BT_NODES_LIBRARY="$<TARGET_FILE:plansys2_epistemic_executor::plansys2_epistemic_bt_nodes>"
  OBSERVATION_BT_NODES_LIBRARY="$<TARGET_FILE:${PROJECT_NAME}_observation_bt_nodes>"
)
target_link_libraries(test_5
  ${PROJECT_NAME}
  ament_index_cpp::ament_index_cpp
  plansys2_domain_expert::plansys2_domain_expert
  plansys2_problem_expert::plansys2_problem_expert
  plansys2_planner::plansys2_planner
  plansys2_epistemic_executor::plansys2_epistemic_executor
)
add_dependencies(test_5 ${PROJECT_NAME}_observation_bt_nodes)
