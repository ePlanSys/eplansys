cmake_minimum_required(VERSION 3.5)
project(plansys2_epistemic_executor)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(plansys2_msgs REQUIRED)
find_package(plansys2_epistemic_msgs REQUIRED)
find_package(plansys2_epddl_grounder REQUIRED)
find_package(plansys2_epistemic_planner REQUIRED)
# Pulled in transitively: the planner library exports it on its interface.
find_package(nlohmann_json 3.10 REQUIRED)

# The policy and its rendering as a behavior tree depend on nothing but the
# message: they are the shared reading of a policy, and keeping them free of
# ROS lets them be tested as the pure transformation they are.
add_library(${PROJECT_NAME}_policy SHARED
  src/policy.cpp
  src/policy_bt.cpp
)
target_include_directories(${PROJECT_NAME}_policy PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(${PROJECT_NAME}_policy PUBLIC ${plansys2_msgs_TARGETS})

# The state, its client, and the behavior tree nodes that consult it.
add_library(${PROJECT_NAME} SHARED
  src/plansys2_epistemic_executor/EpistemicStateNode.cpp
  src/plansys2_epistemic_executor/EpistemicStateClient.cpp
  src/plansys2_epistemic_executor/behavior_tree/epistemic_nodes.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(${PROJECT_NAME}
  PUBLIC
  ${PROJECT_NAME}_policy
  ${plansys2_msgs_TARGETS}
  ${plansys2_epistemic_msgs_TARGETS}
  plansys2_epddl_grounder::plansys2_epddl_grounder
  plansys2_epistemic_planner::plansys2_epistemic_planner
  behaviortree_cpp::behaviortree_cpp
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  ${std_msgs_TARGETS}
)

# The same nodes again, as a BehaviorTree.CPP plugin. A host that loads this
# library gets them registered without linking against this package, which is
# how plansys2_executor runs epistemic trees without depending on any of it.
add_library(plansys2_epistemic_bt_nodes SHARED
  src/plansys2_epistemic_executor/behavior_tree/bt_plugin.cpp
)
target_link_libraries(plansys2_epistemic_bt_nodes PUBLIC ${PROJECT_NAME})
target_compile_definitions(plansys2_epistemic_bt_nodes PRIVATE BT_PLUGIN_EXPORT)

add_executable(epistemic_state_node src/epistemic_state_node.cpp)
target_link_libraries(epistemic_state_node PRIVATE ${PROJECT_NAME} rclcpp::rclcpp)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)
install(DIRECTORY behavior_trees launch
  DESTINATION share/${PROJECT_NAME}
)
install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_policy plansys2_epistemic_bt_nodes
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS epistemic_state_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  add_subdirectory(test)
endif()

ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_libraries(${PROJECT_NAME} ${PROJECT_NAME}_policy plansys2_epistemic_bt_nodes)
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  rclcpp_lifecycle
  std_msgs
  behaviortree_cpp
  plansys2_msgs
  plansys2_epistemic_msgs
  plansys2_epddl_grounder
  plansys2_epistemic_planner
)

ament_package()
