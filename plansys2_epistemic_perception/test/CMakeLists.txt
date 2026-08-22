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
