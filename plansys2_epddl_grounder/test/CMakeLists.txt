# The stand-in for plank has to be executable where the test finds it, and a
# permission bit is not something to trust the source tree to carry.
file(COPY fake_plank.sh
  DESTINATION ${CMAKE_CURRENT_BINARY_DIR}
  FILE_PERMISSIONS
    OWNER_READ OWNER_WRITE OWNER_EXECUTE
    GROUP_READ GROUP_EXECUTE
    WORLD_READ WORLD_EXECUTE
)

ament_add_gtest(epddl_grounder_test epddl_grounder_test.cpp)
target_link_libraries(epddl_grounder_test ${PROJECT_NAME})
target_compile_definitions(epddl_grounder_test PRIVATE
  FAKE_PLANK="${CMAKE_CURRENT_BINARY_DIR}/fake_plank.sh"
  FAKE_PLANK_TASK="${CMAKE_CURRENT_SOURCE_DIR}/fake_task.json"
  EXAMPLES_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../examples"
  LIBRARIES_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../libraries"
)
