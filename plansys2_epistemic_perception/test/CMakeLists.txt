# A grid is a message and nothing else, so these run without a ROS graph.
ament_add_gtest(regions_test regions_test.cpp TIMEOUT 120)
target_link_libraries(regions_test ${PROJECT_NAME}_translation)

ament_add_gtest(translation_test translation_test.cpp TIMEOUT 120)
target_link_libraries(translation_test ${PROJECT_NAME}_translation)

# The node against the two services it calls, stubbed: what the state does with
# an announcement is the state's own test, and what matters here is that a map
# turns into the right call, once.
ament_add_gtest(perception_node_test perception_node_test.cpp TIMEOUT 120)
target_link_libraries(perception_node_test ${PROJECT_NAME} ${lifecycle_msgs_TARGETS})

# The same node against the real epistemic state, holding a real grounded task.
# The stub above accepts whatever it is sent, including a formula naming an atom
# no task declares; only the real state can say whether the call perception
# makes is one a loaded task accepts. The grounded tasks live with the planner
# that produces them, so this reaches across for one at build time.
ament_add_gtest(perception_state_test perception_state_test.cpp TIMEOUT 180)
target_compile_definitions(perception_state_test PRIVATE
  EPISTEMIC_TASK_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../../plansys2_epistemic_planner/test/tasks")
target_link_libraries(perception_state_test ${PROJECT_NAME} ${lifecycle_msgs_TARGETS})
