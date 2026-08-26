cmake_minimum_required(VERSION 3.5)

project(plansys2_tests)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(plansys2_executor REQUIRED)
find_package(plansys2_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(plansys2_core REQUIRED)
find_package(std_msgs REQUIRED)

set(${PROJECT_NAME}_SOURCES
  src/plansys2_tests/test_action_node.cpp
  src/plansys2_tests/execution_logger.cpp
)

add_library(${PROJECT_NAME} SHARED ${${PROJECT_NAME}_SOURCES})
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(${PROJECT_NAME}
  PUBLIC
  ${plansys2_msgs_TARGETS}
  plansys2_executor::plansys2_executor
  rclcpp::rclcpp
)

# What a policy needs and PlanSys2 does not carry: the outcome a performer
# observed, on the port ApplyEpistemicUpdate reads it from. Loaded by the
# executor as a behavior tree plugin, the way a deployment would load its own.
add_library(${PROJECT_NAME}_observation_bt_nodes SHARED
  src/plansys2_tests/behavior_tree/observation_bt_node.cpp
)
target_link_libraries(${PROJECT_NAME}_observation_bt_nodes
  PUBLIC
  behaviortree_cpp::behaviortree_cpp
  rclcpp::rclcpp
  ${std_msgs_TARGETS}
)
target_compile_definitions(${PROJECT_NAME}_observation_bt_nodes PRIVATE BT_PLUGIN_EXPORT)

# The action template that binds the observation. It belongs to no one mission:
# what it adds — a performer's report reaching ApplyEpistemicUpdate — is the
# same wherever a policy senses.
install(DIRECTORY behavior_trees
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_index_cpp REQUIRED)
  find_package(plansys2_domain_expert REQUIRED)
  find_package(plansys2_problem_expert REQUIRED)
  find_package(plansys2_planner REQUIRED)
  find_package(plansys2_pddl_parser REQUIRED)
  find_package(plansys2_epistemic_executor REQUIRED)

  add_subdirectory(test_1)
  add_subdirectory(test_2)
  add_subdirectory(test_3)
  add_subdirectory(test_4)
  add_subdirectory(test_5)
  add_subdirectory(test_6)
endif()

ament_package()
