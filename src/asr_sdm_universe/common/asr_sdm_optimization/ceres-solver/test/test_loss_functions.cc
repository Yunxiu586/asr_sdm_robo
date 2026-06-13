// Smoke test: robust loss functions (Cauchy, Huber).

#include "ceres/ceres.h"
#include "test_utils.h"

struct CostFunctor {
  template <typename T>
  bool operator()(const T* const x, T* residual) const {
    residual[0] = x[0] - T(1.0);
    return true;
  }
};

int main() {
  double x_cauchy = 0.0;
  ceres::Problem cauchy_problem;
  cauchy_problem.AddResidualBlock(
      new ceres::AutoDiffCostFunction<CostFunctor, 1, 1>(new CostFunctor),
      new ceres::CauchyLoss(1.0),
      &x_cauchy);

  ceres::Solver::Options options;
  options.logging_type = ceres::SILENT;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &cauchy_problem, &summary);
  ExpectTrue(summary.IsSolutionUsable(), "cauchy loss solve failed");

  double x_huber = 0.0;
  ceres::Problem huber_problem;
  huber_problem.AddResidualBlock(
      new ceres::AutoDiffCostFunction<CostFunctor, 1, 1>(new CostFunctor),
      new ceres::HuberLoss(0.1),
      &x_huber);
  ceres::Solve(options, &huber_problem, &summary);
  ExpectTrue(summary.IsSolutionUsable(), "huber loss solve failed");
  ExpectNear(x_huber, 1.0, 1e-6, "huber loss result");
  return 0;
}
