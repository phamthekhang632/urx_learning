#pragma once

#include <mc_control/mc_controller.h>

#include "LocalRobot.h"

#include "api.h"

struct UrXLearning_DLLAPI UrXLearning : public mc_control::MCController
{
  UrXLearning(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config);
  bool run() override;
  void reset(const mc_control::ControllerResetData & reset_data) override;

private:
  std::unique_ptr<LocalRobot> local_robot;

  void copyPosture(std::string robot_name, mc_tasks::PostureTask * posture_task);

  void installGripper(const std::string & base_robot, const std::string & gripper_robot);

  void uninstallGripper(std::string base_robot, std::string gripper_robot);
};
