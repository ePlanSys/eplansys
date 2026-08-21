cmake_minimum_required(VERSION 3.5)
project(plansys2_epistemic_planner)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(plansys2_core REQUIRED)
find_package(plansys2_epddl_grounder REQUIRED)
find_package(plansys2_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nlohmann_json 3.10 REQUIRED)

set(EPISTEMIC_SOURCES
  src/action_mapping.cpp
  src/formula.cpp
  src/formula_text.cpp
  src/state.cpp
  src/action.cpp
  src/bisimulation.cpp
  src/product_update.cpp
  src/heuristic.cpp
  src/search.cpp
  src/parser.cpp
  src/policy_plan.cpp
  src/validator.cpp
  src/selection_policy.cpp
  src/epistemic_plan_solver.cpp
)

add_library(${PROJECT_NAME} SHARED ${EPISTEMIC_SOURCES})
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(${PROJECT_NAME}
  PUBLIC
  plansys2_core::plansys2_core
  plansys2_epddl_grounder::plansys2_epddl_grounder
  ${plansys2_msgs_TARGETS}
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  nlohmann_json::nlohmann_json
)
target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

pluginlib_export_plugin_description_file(plansys2_core plansys2_epistemic_planner_plugin.xml)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

# Example action mappings, referenced by the action_mapping parameter.
install(DIRECTORY examples
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  add_subdirectory(test)
endif()

ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  rclcpp_lifecycle
  plansys2_core
  plansys2_epddl_grounder
  plansys2_msgs
  pluginlib
)

ament_package()