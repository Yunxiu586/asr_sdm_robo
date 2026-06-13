#ifndef ASR_SDM_GUIDANCE_PLANNER_PLANNER_PLANNER_BASE_HPP_
#define ASR_SDM_GUIDANCE_PLANNER_PLANNER_PLANNER_BASE_HPP_

#include <types.hpp>
#include <voxel_esdf_map.hpp>

#include <Eigen/Core>

namespace asr_sdm_guidance_planner
{

class PlannerBase
{
public:
  virtual ~PlannerBase() = default;

  virtual PlanResult plan(
    const VoxelEsdfMap & map,
    const Eigen::Vector3d & start,
    const Eigen::Vector3d & goal) = 0;
};

}  // namespace asr_sdm_guidance_planner

#endif  // ASR_SDM_GUIDANCE_PLANNER_PLANNER_PLANNER_BASE_HPP_
