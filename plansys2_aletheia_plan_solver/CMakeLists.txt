cmake_minimum_required(VERSION 3.5)
project(plansys2_aletheia_plan_solver)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(plansys2_core REQUIRED)
find_package(plansys2_msgs REQUIRED)
find_package(plansys2_epistemic_planner REQUIRED)
find_package(nlohmann_json 3.10 REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/plansys2_aletheia_plan_solver/aletheia_plan_solver.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(${PROJECT_NAME}
  PUBLIC
  plansys2_core::plansys2_core
  plansys2_epistemic_planner::plansys2_epistemic_planner
  ${plansys2_msgs_TARGETS}
  rclcpp::rclcpp
  PRIVATE
  pluginlib::pluginlib
  nlohmann_json::nlohmann_json
)

target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

pluginlib_export_plugin_description_file(plansys2_core plansys2_aletheia_plan_solver_plugin.xml)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(FILES plansys2_aletheia_plan_solver_plugin.xml
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
  pluginlib
  plansys2_core
  plansys2_msgs
  plansys2_epistemic_planner
)

ament_package()
