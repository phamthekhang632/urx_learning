#pragma once

#include <mc_control/mc_controller.h>

#include "api.h"
enum ControlState
{
  IDLE = 0,
  MOVE_UP,
  MOVE_DOWN
};

struct UrXLearning_DLLAPI UrXLearning : public mc_control::MCController
{
  UrXLearning(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration &config);
  bool run() override;
  void reset(const mc_control::ControllerResetData &reset_data) override;

private:
  void switch_target();
  ControlState phase_ = IDLE;

  // UR5e
  std::string moveJoint = "wrist_1_joint";
  double jointAngDefault = 0;
  double moveMag = M_PI / 6;
  std::map<std::string, std::vector<double>> defaultPosture = {
      {"shoulder_pan_joint", {M_PI / 2}},
      {"shoulder_lift_joint", {-M_PI / 2}},
      {"elbow_joint", {-M_PI / 2}},
      {"wrist_1_joint", {jointAngDefault}},
      {"wrist_2_joint", {M_PI / 2}},
      {"wrist_3_joint", {0}}};

  // robotiq_arg85
  std::map<std::string, std::vector<double>> gripperClose = {{"finger_joint", {0.0}}};
  std::map<std::string, std::vector<double>> gripperOpen = {{"finger_joint", {0.725}}};
  std::shared_ptr<mc_tasks::PostureTask> gripperPostureTask_;
};
