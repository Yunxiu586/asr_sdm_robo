#ifndef ASR_SDM_TRAJECTORY_PLANNER_COMMON_TYPES_HPP_
#define ASR_SDM_TRAJECTORY_PLANNER_COMMON_TYPES_HPP_

#include <Eigen/Core>

#include <string>
#include <vector>

namespace asr_sdm_guidance_planner
{

struct GridIndex
{
  int x = 0;
  int y = 0;
  int z = 0;

  bool operator==(const GridIndex & other) const
  {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct PlanResult
{
  bool success = false;
  bool optimized = false;
  bool optimized_path_safe = false;
  std::string message;
  std::vector<Eigen::Vector3d> raw_path;
  std::vector<Eigen::Vector3d> trajectory;
};

}  // namespace asr_sdm_guidance_planner

#endif  // ASR_SDM_TRAJECTORY_PLANNER_COMMON_TYPES_HPP_
