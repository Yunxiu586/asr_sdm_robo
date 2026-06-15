cmake_minimum_required(VERSION 3.8)
project(bspline)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

option(BUILD_BSPLINE_TEST_NODE "Build RViz test node for B-spline path generation" ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/non_uniform_bspline.cpp
  src/bspline_path_generator.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}
  Eigen3::Eigen
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
)

if(BUILD_BSPLINE_TEST_NODE)
  find_package(geometry_msgs REQUIRED)
  find_package(nav_msgs REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(visualization_msgs REQUIRED)

  set(TEST_DEPENDENCIES
    rclcpp
    geometry_msgs
    nav_msgs
    std_msgs
    visualization_msgs
  )

  add_executable(bspline_test_node
    test/bspline_test_node.cpp
  )

  target_link_libraries(bspline_test_node
    ${PROJECT_NAME}
  )

  ament_target_dependencies(bspline_test_node
    ${TEST_DEPENDENCIES}
  )
endif()

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

if(BUILD_BSPLINE_TEST_NODE)
  install(
    TARGETS bspline_test_node
    RUNTIME DESTINATION lib/${PROJECT_NAME}
  )

  install(
    DIRECTORY config/
    DESTINATION share/${PROJECT_NAME}/config
  )

  install(
    DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME}/launch
  )

  install(
    DIRECTORY rviz/
    DESTINATION share/${PROJECT_NAME}/rviz
  )
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(Eigen3 rclcpp)
ament_package()
