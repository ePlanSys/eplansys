cmake_minimum_required(VERSION 3.5)
project(plansys2_epddl_grounder)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(plansys2_pddl_parser REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/plansys2_epddl_grounder/epddl_grounder.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(${PROJECT_NAME}
  PUBLIC
  plansys2_pddl_parser::plansys2_pddl_parser
  PRIVATE
  ament_index_cpp::ament_index_cpp
)

add_executable(ground_epddl src/ground_epddl_main.cpp)
target_link_libraries(ground_epddl PRIVATE ${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

# The action-type library the example domains declare, and the EPDDL sources
# of the smallest example. Both are read at run time by path, so they have to
# be installed rather than only present in the source tree.
install(DIRECTORY libraries examples
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ground_epddl
  RUNTIME DESTINATION lib/${PROJECT_NAME}
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
ament_export_dependencies(ament_index_cpp plansys2_pddl_parser)

ament_package()
