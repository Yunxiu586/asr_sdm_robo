#ifndef FRONT_UNIT_FOLLOWING_CONTROLLER_HPP_
#define FRONT_UNIT_FOLLOWING_CONTROLLER_HPP_

#include "types.hpp"

namespace asr
{

class FrontUnitFollowingController
{
public:
  explicit FrontUnitFollowingController(const RobotParameters & params);

  JointVelocity computeJointVelocity(const HeadCommand & head_cmd, const JointState & state) const;

private:
  RobotParameters params_;
};

}  // namespace asr

#endif  // FRONT_UNIT_FOLLOWING_CONTROLLER_HPP_
