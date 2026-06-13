cmake_minimum_required(VERSION 3.8)
project(asr_sdm_trajectory_visualizer)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3 -g")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(bspline REQUIRED)
find_package(asr_sdm_local_path_modifier REQUIRED)
find_package(asr_sdm_esdf_map REQUIRED)
find_package(asr_sdm_trajectory_generator REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/planning_visualization.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  geometry_msgs
  visualization_msgs
  bspline
  asr_sdm_local_path_modifier
  asr_sdm_trajectory_generator
)

# NOTE:
# asr_sdm_esdf_map currently exports a broken imported target
# (asr_sdm_esdf_map::obj_generator). Using ament_target_dependencies() for it
# triggers a CMake error even though this library only needs headers.
if(DEFINED asr_sdm_esdf_map_INCLUDE_DIRS)
  target_include_directories(${PROJECT_NAME} PUBLIC ${asr_sdm_esdf_map_INCLUDE_DIRS})
endif()

if(TARGET Eigen3::Eigen)
  target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)
else()
  target_include_directories(${PROJECT_NAME} PUBLIC ${EIGEN3_INCLUDE_DIR})
endif()

target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp
  std_msgs
  geometry_msgs
  visualization_msgs
  bspline
  asr_sdm_local_path_modifier
  asr_sdm_esdf_map
  asr_sdm_trajectory_generator
)

ament_package()
