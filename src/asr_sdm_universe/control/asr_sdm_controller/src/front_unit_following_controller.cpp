#include "front_unit_following/front_unit_following_controller.hpp"

#include <cmath>

namespace asr
{

struct HeadCommand
{
  double linear_velocity;
  double angular_velocity;
};

struct JointState
{
  std::vector<double> phi;
};

struct JointVelocity
{
  std::vector<double> phi_dot;
};

struct ScrewVelocity
{
  std::vector<double> theta_dot;
};

struct RobotParameters
{
  double link_length;
  double screw_radius;

  std::vector<double> alpha;
};

FrontUnitFollowingController::FrontUnitFollowingController(const RobotParameters & params)
: params_(params)
{
}

JointVelocity FrontUnitFollowingController::computeJointVelocity(
  const HeadCommand & head_cmd, const JointState & state) const
{
  const double L = params_.link_length;

  const double v1 = head_cmd.linear_velocity;
  const double w1 = head_cmd.angular_velocity;

  JointVelocity output;

  const size_t num_joints = state.phi.size();

  output.phi_dot.resize(num_joints, 0.0);

  if (num_joints == 0) {
    return output;
  }

  std::vector<double> v_bar(num_joints + 1, 0.0);
  std::vector<double> psi_dot(num_joints + 1, 0.0);

  v_bar[0] = v1;
  psi_dot[0] = w1;

  for (size_t i = 0; i < num_joints; ++i) {
    const double phi = state.phi[i];

    const double next_psi_dot =
      -(2.0 / L) * v_bar[i] * std::sin(phi) - psi_dot[i] * (std::cos(phi) + 1.0);

    output.phi_dot[i] = next_psi_dot - psi_dot[i];

    psi_dot[i + 1] = next_psi_dot;

    v_bar[i + 1] = v_bar[i] * std::cos(phi) - (L / 2.0) * psi_dot[i] * std::sin(phi);
  }

  return output;
}

}  // namespace asr