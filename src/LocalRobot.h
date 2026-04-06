#pragma once

// #include <mc_control/mc_controller.h>

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

struct LocalRobot
{
  LocalRobot(mc_control::MCController & controller,
             const mc_rbdyn::RobotModule & robot,
             const mc_rbdyn::RobotModule & tool,
             const std::string robot_surface,
             const std::string tool_surface);

  std::string name()
  {
    return name_;
  }

  std::string name_tool()
  {
    return name_tool_;
  }

  mc_rbdyn::RobotModule module()
  {
    return module_;
  }

  mc_rbdyn::RobotModule module_tool()
  {
    return module_tool_;
  }

  std::shared_ptr<mc_tasks::PostureTask> posture_task()
  {
    return posture_task_;
  }

  std::shared_ptr<mc_tasks::PostureTask> posture_task_tool()
  {
    return posture_task_tool_;
  }

  ToolState tool_state()
  {
    return tool_state_;
  }

  void tool_state(ToolState s)
  {
    tool_state_ = s;
  }

  SyncState sync_state()
  {
    return sync_state_;
  }

  void sync_state(SyncState s)
  {
    sync_state_ = s;
  }

private:
  std::string name_{};
  std::string name_tool_{};
  mc_rbdyn::RobotModule module_;
  mc_rbdyn::RobotModule module_tool_;
  std::shared_ptr<mc_tasks::PostureTask> posture_task_{};
  std::shared_ptr<mc_tasks::PostureTask> posture_task_tool_{};
  ToolState tool_state_ = ToolState::IDLE;
  SyncState sync_state_ = SyncState::IDLE;
};

LocalRobot::LocalRobot(mc_control::MCController & controller,
                       const mc_rbdyn::RobotModule & robot,
                       const mc_rbdyn::RobotModule & tool,
                       const std::string robot_surface,
                       const std::string tool_surface)
: name_(robot.name), module_(robot),
  module_tool_(
      robot.connect(tool,
                    robot_surface,
                    tool_surface,
                    "",
                    mc_rbdyn::RobotModule::ConnectionParameters{}.X_other_connection(sva::RotZ(mc_rtc::constants::PI))))
{
  // TODO: generalize connection box
  module_tool_.name = name_ + "_" + tool.name;
  name_tool_ = module_tool_.name;
  mc_rtc::log::info("Attached {} to {} -> {}", tool.name, module_.name, module_tool_.name);

  controller.loadRobot(module_tool_, module_tool_.name);
  controller.gui()->removeElement({"Robots"}, module_tool_.name);

  posture_task_ = std::make_shared<mc_tasks::PostureTask>(controller.solver(), controller.robots().robotIndex(name_));
  controller.solver().addTask(posture_task_);
  posture_task_tool_ =
      std::make_shared<mc_tasks::PostureTask>(controller.solver(), controller.robots().robotIndex(name_tool_));
  controller.solver().addTask(posture_task_tool_);
};
