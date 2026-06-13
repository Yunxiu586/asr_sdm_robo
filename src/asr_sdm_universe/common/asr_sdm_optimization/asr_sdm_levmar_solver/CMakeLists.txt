cmake_minimum_required(VERSION 3.8)
project(asr_sdm_levmar_solver)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(BLAS REQUIRED)
find_package(LAPACK REQUIRED)

set(LEVMAR_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/src/levmar")

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

add_library(levmar_core STATIC
  ${LEVMAR_ROOT}/lm.c
  ${LEVMAR_ROOT}/Axb.c
  ${LEVMAR_ROOT}/misc.c
  ${LEVMAR_ROOT}/lmlec.c
  ${LEVMAR_ROOT}/lmbc.c
  ${LEVMAR_ROOT}/lmblec.c
  ${LEVMAR_ROOT}/lmbleic.c
)

target_include_directories(levmar_core PUBLIC
  $<BUILD_INTERFACE:${LEVMAR_ROOT}>
)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  target_compile_options(levmar_core PRIVATE
    -Wno-array-parameter
    -Wno-unused-but-set-variable
    -Wno-implicit-fallthrough
  )
endif()

target_link_libraries(levmar_core PUBLIC
  LAPACK::LAPACK
  BLAS::BLAS
)

add_library(${PROJECT_NAME} SHARED
  src/levmar_wrapper.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME} PUBLIC
  levmar_core
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS levmar_core ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  add_executable(levmar_test test/levmar_test.cpp)
  target_link_libraries(levmar_test ${PROJECT_NAME})
  target_include_directories(levmar_test PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )

  install(TARGETS levmar_test
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(BLAS LAPACK)
ament_package()
