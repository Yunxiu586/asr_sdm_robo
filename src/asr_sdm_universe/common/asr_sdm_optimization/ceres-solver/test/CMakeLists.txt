# Lightweight smoke tests for the vendored Ceres library (no googletest/abseil).

function(ceres_add_test NAME)
  add_executable(${NAME} ${NAME}.cc)
  target_include_directories(${NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
  target_link_libraries(${NAME} PRIVATE Ceres::ceres)
  add_test(NAME ${NAME} COMMAND ${NAME})
endfunction()

ceres_add_test(test_autodiff_solve)
ceres_add_test(test_dense_schur)
ceres_add_test(test_sparse_cholesky)
ceres_add_test(test_local_parameterization)
ceres_add_test(test_loss_functions)
