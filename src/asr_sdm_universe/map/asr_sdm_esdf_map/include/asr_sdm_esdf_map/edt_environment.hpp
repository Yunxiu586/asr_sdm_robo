// Copyright (c) Amphibious Robotics.
// Topological replanning FSM implementation.

#ifndef _EDT_ENVIRONMENT_HPP_
#define _EDT_ENVIRONMENT_HPP_

#include <Eigen/Eigen>
#include <asr_sdm_esdf_map/esdf_map.hpp>
#include <asr_sdm_esdf_map/obj_predictor.hpp>

#include <iostream>
#include <utility>

namespace amprobo
{
class EDTEnvironment
{
private:
  /* data */
  ObjPrediction obj_prediction_;
  ObjScale obj_scale_;
  double resolution_inv_;
  double distToBox(int idx, const Eigen::Vector3d & pos, const double & time);
  double minDistToAllBox(const Eigen::Vector3d & pos, const double & time);

public:
  EDTEnvironment(/* args */) {}
  ~EDTEnvironment() {}

  ESDFMap::Ptr esdf_map_;

  void init();
  void setMap(ESDFMap::Ptr map);
  void setObjPrediction(ObjPrediction prediction);
  void setObjScale(ObjScale scale);
  void getSurroundDistance(Eigen::Vector3d pts[2][2][2], double dists[2][2][2]);
  void interpolateTrilinear(
    double values[2][2][2], const Eigen::Vector3d & diff, double & value, Eigen::Vector3d & grad);
  void evaluateEDTWithGrad(
    const Eigen::Vector3d & pos, double time, double & dist, Eigen::Vector3d & grad);
  double evaluateCoarseEDT(Eigen::Vector3d & pos, double time);
  void getMapRegion(Eigen::Vector3d & ori, Eigen::Vector3d & size)
  {
    esdf_map_->getRegion(ori, size);
  }

  typedef std::shared_ptr<EDTEnvironment> Ptr;
};

}  // namespace amprobo

#endif