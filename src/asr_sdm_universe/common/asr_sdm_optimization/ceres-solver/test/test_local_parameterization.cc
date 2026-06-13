// Smoke test: LocalParameterization API (used by camera_model and vins_estimator).

#include "ceres/ceres.h"
#include "ceres/rotation.h"
#include "test_utils.h"

struct PointRotationError {
  PointRotationError(double px, double py, double pz) : px_(px), py_(py), pz_(pz) {}

  template <typename T>
  bool operator()(const T* const q, T* residuals) const {
    // Parameter block uses Eigen order [x, y, z, w]; rotation.h uses [w, x, y, z].
    T q_ceres[4] = {q[3], q[0], q[1], q[2]};
    T point[3] = {T(px_), T(py_), T(pz_)};
    T rotated[3];
    ceres::QuaternionRotatePoint(q_ceres, point, rotated);
    residuals[0] = rotated[0] - T(px_);
    residuals[1] = rotated[1] - T(py_);
    residuals[2] = rotated[2] - T(pz_);
    return true;
  }

  double px_, py_, pz_;
};

int main() {
  // Identity quaternion in Ceres order: [x, y, z, w].
  double quaternion[4] = {0.0, 0.0, 0.0, 1.0};

  ceres::Problem problem;
  ceres::LocalParameterization* quaternion_parameterization =
      new ceres::QuaternionParameterization();
  problem.AddParameterBlock(quaternion, 4, quaternion_parameterization);
  problem.AddResidualBlock(
      new ceres::AutoDiffCostFunction<PointRotationError, 3, 4>(
          new PointRotationError(1.0, 0.0, 0.0)),
      nullptr,
      quaternion);

  ceres::Solver::Options options;
  options.logging_type = ceres::SILENT;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  ExpectTrue(summary.IsSolutionUsable(), "local parameterization solve failed");
  ExpectNear(summary.final_cost, 0.0, 1e-9, "identity quaternion residual");
  return 0;
}
