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
  # The planning core is shared with the Aletheia repository, which is where it
  # is developed and from which it is re-synced. Its style is that project's --
  # four-space indent, brace on the same line -- and reformatting it to the ROS
  # convention would make every future diff between the two copies unreadable
  # for no behavioural gain. So the two style linters skip it, and everything
  # else is linted everywhere: ament_copyright still checks these files, which
  # is what keeps the provenance headers honest, and the files original to this
  # package are held to the full ROS style.
  set(ALETHEIA_CORE
    include/plansys2_epistemic_planner/action.hpp
    include/plansys2_epistemic_planner/bisimulation.hpp
    include/plansys2_epistemic_planner/bitset.hpp
    include/plansys2_epistemic_planner/formula.hpp
    include/plansys2_epistemic_planner/heuristic.hpp
    include/plansys2_epistemic_planner/outcome.hpp
    include/plansys2_epistemic_planner/parser.hpp
    include/plansys2_epistemic_planner/product_update.hpp
    include/plansys2_epistemic_planner/search.hpp
    include/plansys2_epistemic_planner/selection_policy.hpp
    include/plansys2_epistemic_planner/state.hpp
    include/plansys2_epistemic_planner/task.hpp
    include/plansys2_epistemic_planner/types.hpp
    include/plansys2_epistemic_planner/validator.hpp
    include/plansys2_epistemic_planner/world_cap_policy.hpp
    src/action.cpp
    src/bisimulation.cpp
    src/formula.cpp
    src/heuristic.cpp
    src/parser.cpp
    src/product_update.cpp
    src/search.cpp
    src/selection_policy.cpp
    src/state.cpp
    src/validator.cpp
  )

  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_cpplint ament_cmake_uncrustify)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_cpplint REQUIRED)
  find_package(ament_cmake_uncrustify REQUIRED)
  ament_cpplint(EXCLUDE ${ALETHEIA_CORE})
  ament_uncrustify(EXCLUDE ${ALETHEIA_CORE})

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
  # In the exported target's link interface, so a package that links this one
  # fails to configure without it — which nothing noticed while every consumer
  # happened to find_package it for its own reasons.
  nlohmann_json
)

ament_package()