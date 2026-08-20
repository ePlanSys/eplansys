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
