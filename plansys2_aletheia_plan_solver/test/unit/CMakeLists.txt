ament_add_gtest(plan_file_test plan_file_test.cpp)
target_link_libraries(plan_file_test ${PROJECT_NAME})

# The fleet mission through the subprocess planner, so both routes to Aletheia
# are exercised on the same scenario. Skips when the binary is absent, since
# nothing here depends on it at build time.
ament_add_gtest(fleet_solver_test fleet_solver_test.cpp TIMEOUT 120)
target_compile_definitions(fleet_solver_test PRIVATE
  EPISTEMIC_TASK_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../../../plansys2_epistemic_planner/test/tasks"
  EPISTEMIC_MAPPING_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../../../plansys2_epistemic_planner/examples/mappings")
target_link_libraries(fleet_solver_test ${PROJECT_NAME})
