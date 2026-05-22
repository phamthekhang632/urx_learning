#include "UrXLearning.h"

#include <mc_rbdyn/RobotLoader.h>
#include <chrono>
#include <thread>

UrXLearning::UrXLearning(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
: mc_control::MCController(std::move(rm), dt)
{
  solver().addConstraintSet(contactConstraint);
  solver().addConstraintSet(kinematicsConstraint);

  solver().setContacts({{}});

  mc_rtc::log::success("UrXLearning init done ");
}

bool UrXLearning::run()
{
  switch(local_robot_->toolState())
  {
    case ToolState::IDLE:
      break;
    case ToolState::DEFAULT:
      uninstallGripper(local_robot_->name(), local_robot_->nameTool());
      break;
    case ToolState::GRIPPER:
      installGripper(local_robot_->name(), local_robot_->nameTool());
      break;
  }
  return mc_control::MCController::run();
}

void UrXLearning::reset(const mc_control::ControllerResetData & reset_data)
{
  mc_control::MCController::reset(reset_data);

  auto gripper_module = mc_rbdyn::RobotLoader::get_robot_module("robotiq_arg85");
  local_robot_ =
      std::make_unique<LocalRobot>(*this, robot().module(), *gripper_module, "wrist_3_link", "robotiq_85_base_link");

  gui()->addElement({"Change Tool"}, mc_rtc::gui::Button(fmt::format("Install gripper"),
                                                         [&]() { local_robot_->setToolState(ToolState::GRIPPER); }));
  gui()->addElement({"Change Tool"}, mc_rtc::gui::Button(fmt::format("Uninstall gripper"),
                                                         [&]() { local_robot_->setToolState(ToolState::DEFAULT); }));
}

CONTROLLER_CONSTRUCTOR("UrXLearning", UrXLearning) // NOLINT(cppcoreguidelines-avoid-non-const-global-variables)

void UrXLearning::copyPosture(const std::string & robot_name, mc_tasks::PostureTask * posture_task)
{
  if(posture_task == nullptr)
  {
    return;
  }

  std::map<std::string, std::vector<double>> current_target;
  const std::vector<std::string> & rjo = robot(robot_name).refJointOrder();

  size_t num_joints = std::min(rjo.size(), size_t(6));
  for(size_t i = 0; i < num_joints; ++i)
  {
    const auto & joint_name = rjo[i];
    auto joint_index = robot(robot_name).jointIndexByName(joint_name);
    current_target[joint_name] = robot(robot_name).mbc().q[joint_index];
  }
  posture_task->target(current_target);
}

void UrXLearning::installGripper(const std::string & base_robot, const std::string & gripper_robot)
{
  if(local_robot_->syncState() == SyncState::IDLE)
  {
    mc_rtc::log::info("Installing gripper - starting sync");
    if(hasRobot(gripper_robot))
    {
      copyPosture(base_robot, local_robot_->postureTaskTool().get());
      local_robot_->setSyncState(SyncState::SYNCING);
    }
  }
  else if(local_robot_->syncState() == SyncState::SYNCING)
  {
    if(local_robot_->postureTaskTool()->eval().norm() < 0.01)
    {
      mc_rtc::log::info("Posture synced");
      gui()->removeElement({"Robots"}, base_robot);
      addRobotToGUI(robot(gripper_robot));
      replaceRobot.signal(base_robot, gripper_robot);
      local_robot_->setSyncState(SyncState::IDLE);
      local_robot_->setToolState(ToolState::IDLE);
    }
  }
}

void UrXLearning::uninstallGripper(const std::string & base_robot, const std::string & gripper_robot)
{
  if(local_robot_->syncState() == SyncState::IDLE)
  {
    mc_rtc::log::info("Uninstalling gripper - starting sync");
    if(hasRobot(base_robot))
    {
      copyPosture(gripper_robot, local_robot_->postureTask().get());
      local_robot_->setSyncState(SyncState::SYNCING);
    }
  }
  else if(local_robot_->syncState() == SyncState::SYNCING)
  {
    if(local_robot_->postureTask()->eval().norm() < 0.01)
    {
      mc_rtc::log::info("Posture synced");
      gui()->removeElement({"Robots"}, gripper_robot);
      addRobotToGUI(robot(base_robot));
      replaceRobot.signal(gripper_robot, base_robot);
      local_robot_->setSyncState(SyncState::IDLE);
      local_robot_->setToolState(ToolState::IDLE);
    }
  }
}
