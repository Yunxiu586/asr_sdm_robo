// Smoke test: DENSE_SCHUR linear solver (used by vins_estimator).

#include "ceres/ceres.h"
#include "ceres/rotation.h"
#include "test_utils.h"

struct ReprojectionError {
  ReprojectionError(double u, double v) : u_(u), v_(v) {}

  template <typename T>
  bool operator()(const T* const camera, const T* const point, T* residuals) const {
    T p[3];
    ceres::AngleAxisRotatePoint(camera, point, p);
    p[0] += camera[3];
    p[1] += camera[4];
    p[2] += camera[5];

    T xp = -p[0] / p[2];
    T yp = -p[1] / p[2];
    const T& focal = camera[6];
    residuals[0] = focal * xp - T(u_);
    residuals[1] = focal * yp - T(v_);
    return true;
  }

  double u_, v_;
};

int main() {
  // One camera, two 3D points, two observations.
  double camera[9] = {0.0, 0.0, 0.0, 0.0, 0.0, 5.0, 500.0, 0.0, 0.0};
  double point_a[3] = {-0.1, 0.0, 10.0};
  double point_b[3] = {0.1, 0.0, 10.0};

  ceres::Problem problem;
  problem.AddResidualBlock(
      new ceres::AutoDiffCostFunction<ReprojectionError, 2, 9, 3>(
          new ReprojectionError(0.0, 0.0)),
      nullptr,
      camera,
      point_a);
  problem.AddResidualBlock(
      new ceres::AutoDiffCostFunction<ReprojectionError, 2, 9, 3>(
          new ReprojectionError(10.0, 0.0)),
      nullptr,
      camera,
      point_b);

  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_SCHUR;
  options.logging_type = ceres::SILENT;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  ExpectTrue(summary.IsSolutionUsable(), "dense schur solve failed");
  ExpectNear(summary.final_cost, 0.0, 1e-3, "dense schur final cost");
  return 0;
}
