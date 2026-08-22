ament_add_gtest(epistemic_parser_test parser_test.cpp TIMEOUT 120)
target_include_directories(epistemic_parser_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_definitions(epistemic_parser_test PRIVATE
  EPISTEMIC_TEST_TASK_DIR="${CMAKE_CURRENT_SOURCE_DIR}/tasks")
target_link_libraries(epistemic_parser_test ${PROJECT_NAME})

# AO* over coin-in-the-box is the slowest case here; the timeout is a hang
# guard, not a performance budget.
ament_add_gtest(epistemic_search_test search_test.cpp TIMEOUT 300)
target_include_directories(epistemic_search_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_definitions(epistemic_search_test PRIVATE
  EPISTEMIC_TEST_TASK_DIR="${CMAKE_CURRENT_SOURCE_DIR}/tasks")
target_link_libraries(epistemic_search_test ${PROJECT_NAME})

ament_add_gtest(epistemic_action_mapping_test action_mapping_test.cpp TIMEOUT 120)
target_include_directories(epistemic_action_mapping_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_definitions(epistemic_action_mapping_test PRIVATE
  EPISTEMIC_TEST_TASK_DIR="${CMAKE_CURRENT_SOURCE_DIR}/tasks"
  EPISTEMIC_EXAMPLE_MAPPING_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../examples/mappings")
target_link_libraries(epistemic_action_mapping_test ${PROJECT_NAME})

ament_add_gtest(epistemic_policy_plan_test policy_plan_test.cpp TIMEOUT 300)
target_include_directories(epistemic_policy_plan_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_definitions(epistemic_policy_plan_test PRIVATE
  EPISTEMIC_TEST_TASK_DIR="${CMAKE_CURRENT_SOURCE_DIR}/tasks")
target_link_libraries(epistemic_policy_plan_test ${PROJECT_NAME})

# The plugin end to end: EPDDL in, a Plan out. Skips itself when plank is not
# built, since the grounder is what makes that route work and it is a run-time
# dependency rather than a build one.
ament_add_gtest(epistemic_plan_solver_test plan_solver_test.cpp TIMEOUT 300)
target_include_directories(epistemic_plan_solver_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_definitions(epistemic_plan_solver_test PRIVATE
  EPISTEMIC_EXAMPLE_MAPPING_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../examples/mappings")
target_link_libraries(epistemic_plan_solver_test ${PROJECT_NAME})

# Reading a formula back from the text a policy carries. It lives here rather
# than with the executor because its counterpart, render_formula, does: the two
# are one grammar written twice, and they drift apart if they are not tested
# together.
ament_add_gtest(epistemic_formula_text_test formula_text_test.cpp TIMEOUT 120)
target_include_directories(epistemic_formula_text_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_definitions(epistemic_formula_text_test PRIVATE
  EPISTEMIC_TASK_DIR="${CMAKE_CURRENT_SOURCE_DIR}/tasks")
target_link_libraries(epistemic_formula_text_test ${PROJECT_NAME})
