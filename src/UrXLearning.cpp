#include "UrXLearning.h"

#include <mc_rbdyn/RobotLoader.h>
#include <chrono>
#include <thread>

UrXLearning::UrXLearning(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
: mc_control::MCController(rm, dt)
{
  solver().addConstraintSet(contactConstraint);
  solver().addConstraintSet(kinematicsConstraint);

  solver().setContacts({{}});

  mc_rtc::log::success("UrXLearning init done ");
}

bool UrXLearning::run()
{
  switch(local_robot->tool_state())
  {
    case ToolState::IDLE:
      break;
    case ToolState::DEFAULT:
      uninstallGripper(local_robot->name(), local_robot->name_tool());
      break;
    case ToolState::GRIPPER:
      installGripper(local_robot->name(), local_robot->name_tool());
      break;
  }
  return mc_control::MCController::run();
}

void UrXLearning::reset(const mc_control::ControllerResetData & reset_data)
{
  mc_control::MCController::reset(reset_data);

  mc_rbdyn::RobotModule gripper_module = *(mc_rbdyn::RobotLoader::get_robot_module("robotiq_arg85"));
  local_robot =
      std::make_unique<LocalRobot>(*this, robot().module(), gripper_module, "wrist_3_link", "robotiq_85_base_link");

  gui()->addElement({"Change Tool"}, mc_rtc::gui::Button(fmt::format("Install gripper"),
                                                         [&]() { local_robot->tool_state(ToolState::GRIPPER); }));
  gui()->addElement({"Change Tool"}, mc_rtc::gui::Button(fmt::format("Uninstall gripper"),
                                                         [&]() { local_robot->tool_state(ToolState::DEFAULT); }));
}

CONTROLLER_CONSTRUCTOR("UrXLearning", UrXLearning)

// TODO: move copyPosture to LocalRobot.h
void UrXLearning::copyPosture(std::string robot_name, mc_tasks::PostureTask * posture_task)
{
  if(!posture_task) return;

  std::map<std::string, std::vector<double>> current_target;
  const std::vector<std::string> & rjo = robot(robot_name).refJointOrder();

  size_t num_joints = std::min(rjo.size(), size_t(6));
  for(size_t i = 0; i < num_joints; ++i)
  {
    const auto & joint_name = rjo[i];
    auto jIndex = robot(robot_name).jointIndexByName(joint_name);
    current_target[joint_name] = robot(robot_name).mbc().q[jIndex];
  }
  posture_task->target(current_target);
}

void UrXLearning::installGripper(const std::string & base_robot, const std::string & gripper_robot)
{
  if(local_robot->sync_state() == SyncState::IDLE)
  {
    mc_rtc::log::info("Installing gripper - starting sync");
    if(hasRobot(gripper_robot))
    {
      copyPosture(base_robot, local_robot->posture_task_tool().get());
      local_robot->sync_state(SyncState::SYNCING);
    }
  }
  else if(local_robot->sync_state() == SyncState::SYNCING)
  {
    if(local_robot->posture_task_tool()->eval().norm() < 0.01)
    {
      mc_rtc::log::info("Posture synced");
      gui()->removeElement({"Robots"}, base_robot);
      addRobotToGUI(robot(gripper_robot));
      replaceRobot.signal(base_robot, gripper_robot);
      local_robot->sync_state(SyncState::IDLE);
      local_robot->tool_state(ToolState::IDLE);
    }
  }
}

void UrXLearning::uninstallGripper(std::string base_robot, std::string gripper_robot)
{
  if(local_robot->sync_state() == SyncState::IDLE)
  {
    mc_rtc::log::info("Uninstalling gripper - starting sync");
    if(hasRobot(base_robot))
    {
      copyPosture(gripper_robot, local_robot->posture_task().get());
      local_robot->sync_state(SyncState::SYNCING);
    }
  }
  else if(local_robot->sync_state() == SyncState::SYNCING)
  {
    if(local_robot->posture_task()->eval().norm() < 0.01)
    {
      mc_rtc::log::info("Posture synced");
      gui()->removeElement({"Robots"}, gripper_robot);
      addRobotToGUI(robot(base_robot));
      replaceRobot.signal(gripper_robot, base_robot);
      local_robot->sync_state(SyncState::IDLE);
      local_robot->tool_state(ToolState::IDLE);
    }
  }
}
