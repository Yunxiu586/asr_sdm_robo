// Smoke test: AutoDiff cost function and basic solver.

#include "ceres/ceres.h"
#include "test_utils.h"

struct CostFunctor {
  template <typename T>
  bool operator()(const T* const x, T* residual) const {
    residual[0] = T(10.0) - x[0];
    return true;
  }
};

int main() {
  double x = 0.5;
  ceres::Problem problem;
  problem.AddResidualBlock(
      new ceres::AutoDiffCostFunction<CostFunctor, 1, 1>(new CostFunctor),
      nullptr,
      &x);

  ceres::Solver::Options options;
  options.logging_type = ceres::SILENT;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  ExpectTrue(summary.IsSolutionUsable(), "solver did not converge");
  ExpectNear(x, 10.0, 1e-6, "autodiff solve result");
  return 0;
}
