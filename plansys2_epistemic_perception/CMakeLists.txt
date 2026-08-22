cmake_minimum_required(VERSION 3.5)
project(plansys2_epistemic_perception)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(plansys2_epistemic_msgs REQUIRED)
find_package(plansys2_epistemic_executor REQUIRED)

# Reading a grid as regions, and regions as formulas. Both halves are pure
# functions of a message and a configuration: no node, no state, no clock, so
# they can be tested against canned grids with no graph and no robot.
add_library(${PROJECT_NAME}_translation SHARED
  src/plansys2_epistemic_perception/regions.cpp
  src/plansys2_epistemic_perception/translation.cpp
)
target_include_directories(${PROJECT_NAME}_translation PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(${PROJECT_NAME}_translation PUBLIC ${nav_msgs_TARGETS})

# The node adds the watching and the reporting, and no translation of its own.
add_library(${PROJECT_NAME} SHARED
  src/plansys2_epistemic_perception/EpistemicPerceptionNode.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(${PROJECT_NAME}
  PUBLIC
  ${PROJECT_NAME}_translation
  ${nav_msgs_TARGETS}
  ${plansys2_epistemic_msgs_TARGETS}
  plansys2_epistemic_executor::plansys2_epistemic_executor
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
)

add_executable(epistemic_perception_node src/epistemic_perception_node.cpp)
target_link_libraries(epistemic_perception_node PRIVATE ${PROJECT_NAME} rclcpp::rclcpp)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)
install(DIRECTORY launch params
  DESTINATION share/${PROJECT_NAME}
)
install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_translation
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS epistemic_perception_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  find_package(lifecycle_msgs REQUIRED)
  add_subdirectory(test)
endif()

ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_libraries(${PROJECT_NAME} ${PROJECT_NAME}_translation)
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  rclcpp_lifecycle
  nav_msgs
  plansys2_epistemic_msgs
  plansys2_epistemic_executor
)

ament_package()
