#ifndef BSPLINE_PATH_GENERATOR_H_
#define BSPLINE_PATH_GENERATOR_H_

#include <bspline/non_uniform_bspline.h>

#include <Eigen/Core>

#include <string>
#include <vector>

namespace bspline
{

struct BsplinePathGeneratorOptions
{
  int order = 3;
  double ts = 0.20;
  double sample_dt = 0.02;
  double min_input_point_spacing = 1.0e-3;
  int max_reallocation_iter = 8;
  bool fallback_to_input_on_failure = true;
  double max_vel = 2.0;
  double max_acc = 3.0;
};

struct BsplinePathResult
{
  bool success = false;
  bool feasible = false;
  int reallocation_iter = 0;
  std::string message;
  std::vector<Eigen::Vector3d> input_path;
  std::vector<Eigen::Vector3d> samples;
  Eigen::MatrixXd control_points;
  double duration = 0.0;
  double mean_vel = 0.0;
  double max_vel = 0.0;
  double mean_acc = 0.0;
  double max_acc = 0.0;
};

// One-shot helper for manager / test code. It intentionally processes one candidate path
// at a time; an upper-level manager can loop or create optimization threads for all topo paths.
class BsplinePathGenerator
{
public:
  BsplinePathGenerator() = default;
  explicit BsplinePathGenerator(const BsplinePathGeneratorOptions & options);

  void setOptions(const BsplinePathGeneratorOptions & options);
  const BsplinePathGeneratorOptions & options() const { return options_; }

  BsplinePathResult generate(const std::vector<Eigen::Vector3d> & candidate_path) const;

private:
  std::vector<Eigen::Vector3d> sanitizePath(const std::vector<Eigen::Vector3d> & path) const;
  bool parameterize(const std::vector<Eigen::Vector3d> & path, Eigen::MatrixXd & ctrl_pts) const;
  std::vector<Eigen::Vector3d> sampleSpline(fast_planner::NonUniformBspline & spline) const;

  BsplinePathGeneratorOptions options_;
};

}  // namespace bspline

#endif  // BSPLINE_PATH_GENERATOR_H_
