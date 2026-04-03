#include "UrXLearning.h"

#include <mc_rbdyn/RobotLoader.h>
#include <chrono>
#include <thread>

UrXLearning::UrXLearning(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
: mc_control::MCController(rm, dt)
{
  solver().addConstraintSet(contactConstraint);
  solver().addConstraintSet(kinematicsConstraint);
  solver().addTask(postureTask);

  solver().setContacts({{}});

  mc_rtc::log::success("UrXLearning init done ");
}

bool UrXLearning::run()
{
  switch(tool_state)
  {
    case ToolState::IDLE:
      break;
    case ToolState::DEFAULT:
      uninstallGripper(base_robot, gripper_robot);
      break;
    case ToolState::GRIPPER:
      installGripper(base_robot, gripper_robot);
      break;
  }
  return mc_control::MCController::run();
}

void UrXLearning::reset(const mc_control::ControllerResetData & reset_data)
{
  mc_control::MCController::reset(reset_data);

  mc_rbdyn::RobotModule ur5e_module = robot().module();
  mc_rbdyn::RobotModule gripper_module = *(mc_rbdyn::RobotLoader::get_robot_module("robotiq_arg85"));
  mc_rbdyn::RobotModule connect = connectModules(ur5e_module, gripper_module, "wrist_3_link", "robotiq_85_base_link");
  loadRobot(connect, connect.name);

  base_robot = robot().name();
  gripper_robot = connect.name;

  posture_task = std::make_shared<mc_tasks::PostureTask>(solver(), robots().robotIndex(connect.name));
  copyPosture(robot().name(), *posture_task);
  posture_task->setGains(postureTask->stiffness(), postureTask->damping());
  solver().addTask(posture_task);

  gui()->addElement({"Change Tool"},
                    mc_rtc::gui::Button(fmt::format("Install gripper"), [&]() { tool_state = ToolState::GRIPPER; }));
  gui()->addElement({"Change Tool"},
                    mc_rtc::gui::Button(fmt::format("Uninstall gripper"), [&]() { tool_state = ToolState::DEFAULT; }));
}

CONTROLLER_CONSTRUCTOR("UrXLearning", UrXLearning)

mc_rbdyn::RobotModule UrXLearning::connectModules(const mc_rbdyn::RobotModule & robot,
                                                  const mc_rbdyn::RobotModule & tool,
                                                  const std::string robot_surface,
                                                  const std::string tool_surface,
                                                  const std::string name_suffix /*= ""*/)
{
  mc_rbdyn::RobotModule connect =
      robot.connect(tool, robot_surface, tool_surface, "",
                    mc_rbdyn::RobotModule::ConnectionParameters{}.X_other_connection(sva::RotZ(mc_rtc::constants::PI)));
  connect.name = robot.name + "_" + (!name_suffix.empty() ? name_suffix : tool.name);
  mc_rtc::log::info("Attached {} to {} -> {}", tool.name, robot.name, connect.name);

  return connect;
}

void UrXLearning::copyPosture(std::string robot_name, mc_tasks::PostureTask & posture_task)
{
  std::map<std::string, std::vector<double>> current_target;
  const std::vector<std::string> & rjo = robot(robot_name).refJointOrder();

  size_t num_joints = std::min(rjo.size(), size_t(6));
  for(size_t i = 0; i < num_joints; ++i)
  {
    const auto & joint_name = rjo[i];
    auto jIndex = robot(robot_name).jointIndexByName(joint_name);
    current_target[joint_name] = robot(robot_name).mbc().q[jIndex];
  }
  posture_task.target(current_target);
}

void UrXLearning::installGripper(const std::string & base_robot, const std::string & gripper_robot)
{
  if(sync_state == SyncState::IDLE)
  {
    mc_rtc::log::info("Installing gripper - starting sync");
    if(hasRobot(gripper_robot))
    {
      copyPosture(base_robot, *posture_task);
      sync_state = SyncState::SYNCING;
    }
  }
  else if(sync_state == SyncState::SYNCING)
  {
    if(posture_task->eval().norm() < 0.01)
    {
      mc_rtc::log::info("Posture synced");
      gui()->removeElement({"Robots"}, base_robot);
      addRobotToGUI(robot(gripper_robot));
      replaceRobot.signal(base_robot, gripper_robot, "");
      sync_state = SyncState::IDLE;
      tool_state = ToolState::IDLE;
    }
  }
}

void UrXLearning::uninstallGripper(std::string base_robot, std::string gripper_robot)
{
  if(sync_state == SyncState::IDLE)
  {
    mc_rtc::log::info("Uninstalling gripper - starting sync");
    if(hasRobot(base_robot))
    {
      copyPosture(gripper_robot, *postureTask);
      sync_state = SyncState::SYNCING;
    }
  }
  else if(sync_state == SyncState::SYNCING)
  {
    if(postureTask->eval().norm() < 0.01)
    {
      mc_rtc::log::info("Posture synced");
      gui()->removeElement({"Robots"}, gripper_robot);
      addRobotToGUI(robot(base_robot));
      replaceRobot.signal(gripper_robot, base_robot, "");
      sync_state = SyncState::IDLE;
      tool_state = ToolState::IDLE;
    }
  }
}
