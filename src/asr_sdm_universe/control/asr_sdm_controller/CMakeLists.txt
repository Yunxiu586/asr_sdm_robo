cmake_minimum_required(VERSION 3.8)
project(asr_sdm_controller)

# Default to RelWithDebInfo so plain `colcon build --packages-select asr_sdm_controller`
# still gets -O2 -g instead of an unoptimized, stripped binary.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(asr_sdm_control_msgs REQUIRED)
find_package(asr_sdm_hardware_msgs REQUIRED)
# find_package(periphery REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development NumPy)
# Find libgpiod
# find_package(PkgConfig REQUIRED)
# pkg_check_modules(GPIOD REQUIRED libgpiod)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# find_library(MRAA_LIB NAMES mraa PATHS /usr/local/include)
# find_library(PERIPHERY_LIB NAMES periphery PATHS /usr/local/include)

include_directories(
  include
  # ${GPIOD_INCLUDE_DIRS}
  /usr/local/include
)

# add_executable(mcp_can src/mcp_can.cpp)
# target_link_libraries(mcp_can ${MRAA_LIB})
# ament_target_dependencies(mcp_can rclcpp)

add_executable(asr_sdm_controller src/asr_sdm_controller_node.cpp)
# target_link_libraries(asr_sdm_controller periphery)
target_link_libraries(asr_sdm_controller PUBLIC
  ${asr_sdm_control_msgs_TARGETS}
  ${asr_sdm_hardware_msgs_TARGETS}
  ${std_msgs_TARGETS}
  rclcpp::rclcpp
  # periphery
)
# ament_target_dependencies(asr_sdm_controller rclcpp std_msgs asr_sdm_control_msgs asr_sdm_hardware_msgs)

add_executable(front_unit_following_controller_test_2d
  test/front_unit_following_controller_test_2d.cpp
  src/front_unit_following_controller.cpp
)
target_link_libraries(front_unit_following_controller_test_2d PUBLIC
  Python3::Python
  Python3::NumPy
)
target_include_directories(front_unit_following_controller_test_2d PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/test
)
# 抑制 vendored matplotlibcpp.h 中已弃用的 Python C API 警告
target_compile_options(front_unit_following_controller_test_2d PRIVATE -Wno-deprecated-declarations)

add_executable(realtime_front_unit_controller
  src/realtime_front_unit_controller_node.cpp
  src/front_unit_following_controller.cpp
)
target_link_libraries(realtime_front_unit_controller PUBLIC
  ${asr_sdm_control_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${std_msgs_TARGETS}
  rclcpp::rclcpp
)

add_executable(realtime_controller_visualizer src/realtime_controller_visualizer_node.cpp)
target_link_libraries(realtime_controller_visualizer PUBLIC
  ${std_msgs_TARGETS}
  rclcpp::rclcpp
  Python3::Python
  Python3::NumPy
)
target_include_directories(realtime_controller_visualizer PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/test
)
target_compile_options(realtime_controller_visualizer PRIVATE -Wno-deprecated-declarations)

install(TARGETS
  asr_sdm_controller
  front_unit_following_controller_test_2d
  realtime_front_unit_controller
  realtime_controller_visualizer
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
