cmake_minimum_required(VERSION 3.8)
project(eplansys_demo)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(plansys2_msgs REQUIRED)
find_package(plansys2_executor REQUIRED)
find_package(plansys2_planner REQUIRED)
find_package(plansys2_problem_expert REQUIRED)
find_package(plansys2_domain_expert REQUIRED)
find_package(plansys2_pddl_parser REQUIRED)

add_executable(corridor_actions src/corridor_actions.cpp)
target_link_libraries(corridor_actions
  plansys2_executor::plansys2_executor
  rclcpp::rclcpp
)
target_compile_definitions(corridor_actions PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

add_executable(corridor_mission src/corridor_mission.cpp)
target_link_libraries(corridor_mission
  plansys2_executor::plansys2_executor
  plansys2_planner::plansys2_planner
  plansys2_problem_expert::plansys2_problem_expert
  plansys2_domain_expert::plansys2_domain_expert
  plansys2_pddl_parser::plansys2_pddl_parser
  ${plansys2_msgs_TARGETS}
  rclcpp::rclcpp
)
target_compile_definitions(corridor_mission PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

install(TARGETS corridor_actions corridor_mission
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch params pddl epddl
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
