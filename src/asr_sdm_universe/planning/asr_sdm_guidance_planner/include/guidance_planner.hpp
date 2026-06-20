#ifndef ASR_SDM_GUIDANCE_PLANNER_GUIDANCE_PLANNER_HPP_
#define ASR_SDM_GUIDANCE_PLANNER_GUIDANCE_PLANNER_HPP_

#include <Eigen/Core>
#include <astar_3d_planner.hpp>
#include <lbfgs_path_optimizer.hpp>
#include <sphere_corridor.hpp>
#include <voxel_esdf_map.hpp>

#include <string>
#include <vector>

namespace asr_sdm_guidance_planner
{

struct GuidancePlannerOptions
{
  Astar3dOptions astar;
  SphereCorridorOptions corridor;
  LbfgsPathOptimizerOptions optimizer;

  bool use_optimized_only_if_safe = true;
  bool project_start_goal_to_safe = true;
  double safe_point_search_radius = 1.5;
};

struct GuidancePlannerResult
{
  bool success = false;
  bool used_fallback = false;
  bool start_projected = false;
  bool goal_projected = false;

  std::string message;
  std::string timing_summary;
  std::string start_projection_status;
  std::string goal_projection_status;

  Eigen::Vector3d requested_start = Eigen::Vector3d::Zero();
  Eigen::Vector3d requested_goal = Eigen::Vector3d::Zero();
  Eigen::Vector3d planning_start = Eigen::Vector3d::Zero();
  Eigen::Vector3d planning_goal = Eigen::Vector3d::Zero();

  std::vector<Eigen::Vector3d> raw_path;
  std::vector<Eigen::Vector3d> corridor_waypoints;
  std::vector<CorridorSphere> corridor_spheres;
  std::vector<Eigen::Vector3d> final_waypoints;
};

class GuidancePlanner
{
public:
  explicit GuidancePlanner(const GuidancePlannerOptions & options = GuidancePlannerOptions());

  void setOptions(const GuidancePlannerOptions & options);
  const GuidancePlannerOptions & options() const { return options_; }

  GuidancePlannerResult plan(
    const MapQueryInterface & map, const Eigen::Vector3d & start, const Eigen::Vector3d & goal);

private:
  double corridorRequiredClearance() const;

  bool findNearestSafePlanningPoint(
    const MapQueryInterface & map, const Eigen::Vector3d & seed, const std::string & label,
    Eigen::Vector3d & safe_point, std::string & status) const;

  GuidancePlannerOptions options_;
  Astar3dPlanner astar_planner_;
  SphereCorridorGenerator corridor_generator_;
  LbfgsPathOptimizer optimizer_;
};

}  // namespace asr_sdm_guidance_planner

#endif  // ASR_SDM_GUIDANCE_PLANNER_GUIDANCE_PLANNER_HPP_
