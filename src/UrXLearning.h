#pragma once

#include <mc_control/mc_controller.h>

#include "api.h"

enum class ToolState
{
  IDLE = 0,
  DEFAULT,
  GRIPPER
};

enum class SyncState
{
  IDLE = 0,
  SYNCING
};

struct UrXLearning_DLLAPI UrXLearning : public mc_control::MCController
{
  UrXLearning(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config);
  bool run() override;
  void reset(const mc_control::ControllerResetData & reset_data) override;

private:
  ToolState tool_state = ToolState::IDLE;
  SyncState sync_state = SyncState::IDLE;
  std::string base_robot = "";
  std::string gripper_robot = "";
  std::shared_ptr<mc_tasks::PostureTask> posture_task;

  mc_rbdyn::RobotModule connectModules(const mc_rbdyn::RobotModule & robot,
                                       const mc_rbdyn::RobotModule & tool,
                                       const std::string robot_surface,
                                       const std::string tool_surface,
                                       const std::string name_suffix = "");

  void copyPosture(std::string robot_name, mc_tasks::PostureTask & posture_task);

  void installGripper(const std::string & base_robot, const std::string & gripper_robot);

  void uninstallGripper(std::string base_robot, std::string gripper_robot);
};
