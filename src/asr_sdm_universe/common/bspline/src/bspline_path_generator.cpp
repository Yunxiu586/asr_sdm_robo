#include <bspline/bspline_path_generator.h>

#include <algorithm>
#include <cmath>
#include <limits>

namespace bspline
{

BsplinePathGenerator::BsplinePathGenerator(const BsplinePathGeneratorOptions & options)
{
  setOptions(options);
}

void BsplinePathGenerator::setOptions(const BsplinePathGeneratorOptions & options)
{
  options_ = options;
  if (options_.order != 3) {
    options_.order = 3;
  }
  options_.ts = std::max(1.0e-6, options_.ts);
  options_.sample_dt = std::max(1.0e-6, options_.sample_dt);
  options_.min_input_point_spacing = std::max(0.0, options_.min_input_point_spacing);
  options_.max_reallocation_iter = std::max(0, options_.max_reallocation_iter);
  options_.max_vel = std::max(1.0e-6, options_.max_vel);
  options_.max_acc = std::max(1.0e-6, options_.max_acc);
}

BsplinePathResult BsplinePathGenerator::generate(const std::vector<Eigen::Vector3d> & candidate_path) const
{
  BsplinePathResult result;
  result.input_path = sanitizePath(candidate_path);
  if (result.input_path.size() < 2U) {
    result.message = "input candidate has fewer than 2 valid points";
    return result;
  }

  Eigen::MatrixXd ctrl_pts;
  if (!parameterize(result.input_path, ctrl_pts)) {
    result.message = "parameterizeToBspline failed";
    if (options_.fallback_to_input_on_failure) {
      result.samples = result.input_path;
    }
    return result;
  }

  fast_planner::NonUniformBspline spline(ctrl_pts, options_.order, options_.ts);
  spline.setPhysicalLimits(options_.max_vel, options_.max_acc);

  bool feasible = spline.checkFeasibility(false);
  int realloc_iter = 0;
  while (!feasible && realloc_iter < options_.max_reallocation_iter) {
    spline.reallocateTime(false);
    ++realloc_iter;
    feasible = spline.checkFeasibility(false);
  }

  result.reallocation_iter = realloc_iter;
  result.feasible = feasible;
  result.control_points = spline.getControlPoint();
  result.duration = spline.getTimeSum();
  spline.getMeanAndMaxVel(result.mean_vel, result.max_vel);
  spline.getMeanAndMaxAcc(result.mean_acc, result.max_acc);

  if (!feasible && !options_.fallback_to_input_on_failure) {
    result.message = "dynamic feasibility failed";
    return result;
  }

  result.samples = sampleSpline(spline);
  if (result.samples.size() < 2U) {
    result.message = "sampling failed";
    if (options_.fallback_to_input_on_failure) {
      result.samples = result.input_path;
    } else {
      return result;
    }
  }

  result.success = result.samples.size() >= 2U;
  result.message = result.success ? "ok" : "failed";
  return result;
}

std::vector<Eigen::Vector3d> BsplinePathGenerator::sanitizePath(
  const std::vector<Eigen::Vector3d> & path) const
{
  std::vector<Eigen::Vector3d> result;
  result.reserve(path.size());
  for (const auto & point : path) {
    if (!std::isfinite(point.x()) || !std::isfinite(point.y()) || !std::isfinite(point.z())) {
      continue;
    }
    if (result.empty() || (point - result.back()).norm() >= options_.min_input_point_spacing) {
      result.push_back(point);
    }
  }
  return result;
}

bool BsplinePathGenerator::parameterize(
  const std::vector<Eigen::Vector3d> & path,
  Eigen::MatrixXd & ctrl_pts) const
{
  if (path.size() < 2U) {
    return false;
  }

  std::vector<Eigen::Vector3d> start_end_derivative;
  start_end_derivative.reserve(4);
  start_end_derivative.push_back(Eigen::Vector3d::Zero());
  start_end_derivative.push_back(Eigen::Vector3d::Zero());
  start_end_derivative.push_back(Eigen::Vector3d::Zero());
  start_end_derivative.push_back(Eigen::Vector3d::Zero());

  fast_planner::NonUniformBspline::parameterizeToBspline(
    options_.ts, path, start_end_derivative, ctrl_pts);

  return ctrl_pts.rows() >= options_.order + 1 && ctrl_pts.cols() == 3 && ctrl_pts.allFinite();
}

std::vector<Eigen::Vector3d> BsplinePathGenerator::sampleSpline(
  fast_planner::NonUniformBspline & spline) const
{
  std::vector<Eigen::Vector3d> samples;
  const double duration = spline.getTimeSum();
  if (!std::isfinite(duration) || duration <= 1.0e-6) {
    return samples;
  }

  const int reserve_num = std::max(2, static_cast<int>(std::ceil(duration / options_.sample_dt)) + 2);
  samples.reserve(static_cast<std::size_t>(reserve_num));

  double last_t = 0.0;
  for (double t = 0.0; t <= duration + 1.0e-9; t += options_.sample_dt) {
    Eigen::VectorXd p = spline.evaluateDeBoorT(std::min(t, duration));
    if (p.size() >= 3 && p.allFinite()) {
      samples.push_back(Eigen::Vector3d(p(0), p(1), p(2)));
      last_t = t;
    }
  }

  if (samples.empty() || std::fabs(last_t - duration) > 1.0e-6) {
    Eigen::VectorXd p = spline.evaluateDeBoorT(duration);
    if (p.size() >= 3 && p.allFinite()) {
      samples.push_back(Eigen::Vector3d(p(0), p(1), p(2)));
    }
  }

  return samples;
}

}  // namespace bspline
