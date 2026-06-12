// Smoke test: SPARSE_NORMAL_CHOLESKY linear solver (used by pose_graph).

#include "ceres/ceres.h"
#include "test_utils.h"

struct TranslationError {
  TranslationError(double dx, double dy) : dx_(dx), dy_(dy) {}

  template <typename T>
  bool operator()(const T* const t_i, const T* const t_j, T* residuals) const {
    residuals[0] = (t_j[0] - t_i[0]) - T(dx_);
    residuals[1] = (t_j[1] - t_i[1]) - T(dy_);
    return true;
  }

  double dx_, dy_;
};

int main() {
  double t0[2] = {0.0, 0.0};
  double t1[2] = {1.0, 0.0};
  double t2[2] = {2.0, 0.0};

  ceres::Problem problem;
  problem.AddParameterBlock(t0, 2);
  problem.AddParameterBlock(t1, 2);
  problem.AddParameterBlock(t2, 2);
  problem.SetParameterBlockConstant(t0);

  problem.AddResidualBlock(
      new ceres::AutoDiffCostFunction<TranslationError, 2, 2, 2>(
          new TranslationError(1.0, 0.0)),
      nullptr,
      t0,
      t1);
  problem.AddResidualBlock(
      new ceres::AutoDiffCostFunction<TranslationError, 2, 2, 2>(
          new TranslationError(1.0, 0.0)),
      nullptr,
      t1,
      t2);

  ceres::Solver::Options options;
  options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  options.logging_type = ceres::SILENT;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  ExpectTrue(summary.IsSolutionUsable(), "sparse cholesky solve failed");
  ExpectNear(t1[0], 1.0, 1e-6, "t1.x");
  ExpectNear(t2[0], 2.0, 1e-6, "t2.x");
  return 0;
}
