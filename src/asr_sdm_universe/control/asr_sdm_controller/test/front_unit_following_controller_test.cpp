#include "asr_sdm_controller/front_unit_following_controller.hpp"

#include "asr_sdm_controller/robot_model.hpp"

#include <iostream>

using namespace asr;

int main()
{
  RobotParameters params;

  params.link_length = 0.226;
  params.screw_radius = 0.075;

  params.alpha = {-M_PI / 4.0, M_PI / 4.0, -M_PI / 4.0, M_PI / 4.0};

  FrontUnitFollowingController controller(params);
  RobotModel robot_model(params);

  HeadCommand cmd;
  cmd.linear_velocity = 0.2;
  cmd.angular_velocity = 0.4;

  JointState state;
  state.phi = {0.1, 0.05, -0.03};

  auto joint_vel = controller.computeJointVelocity(cmd, state);

  auto screw_vel = robot_model.computeScrewVelocity(cmd, state, joint_vel);

  std::cout << "Joint velocities:" << std::endl;

  for (double v : joint_vel.phi_dot) {
    std::cout << v << std::endl;
  }

  std::cout << "Screw velocities:" << std::endl;

  for (double v : screw_vel.theta_dot) {
    std::cout << v << std::endl;
  }

  return 0;
}