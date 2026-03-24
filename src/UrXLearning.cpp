#include "UrXLearning.h"

#include <mc_rbdyn/RobotLoader.h>
#include <chrono>
#include <thread>

UrXLearning::UrXLearning(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration &config)
    : mc_control::MCController(rm, dt)
{
  solver().addConstraintSet(contactConstraint);
  solver().addConstraintSet(kinematicsConstraint);
  if (mode != POSTURE)
  {
    postureTask->setGains(0.5, 1.4);
    postureTask->weight(1);
    solver().addTask(postureTask);
  }
  solver().setContacts({{}});

  mc_rtc::log::success("UrXLearning init done ");
}

bool UrXLearning::run()
{
  if (mode == POSTURE)
  {
  }
  else if (postureTask->eval().norm() < 0.02 && postureTask->speed().norm() < 0.01)
  {
    if (mode == OSCILLATE)
      switch_target();
    else
      store_target();
  }
  return mc_control::MCController::run();
}

void UrXLearning::reset(const mc_control::ControllerResetData & reset_data)
{
  mc_control::MCController::reset(reset_data);

  auto robot_module = robot().module();
  auto connect_rm = mc_rbdyn::RobotLoader::get_robot_module("robotiq_arg85");
  // Connect two robot modules
  auto connect = robot_module.connect(
      *connect_rm, "wrist_3_link", "robotiq_85_base_link", "",
      mc_rbdyn::RobotModule::ConnectionParameters{}.X_other_connection(sva::RotZ(mc_rtc::constants::PI)));
  connect.name = "ur5e_gripper";
  loadRobot(connect, connect.name);
  gui()->removeElement({"Robots"}, robot().name());
  postureTask_ = std::make_shared<mc_tasks::PostureTask>(solver(), robots().robotIndex(connect.name));
  solver().addTask(postureTask_);
}

CONTROLLER_CONSTRUCTOR("UrXLearning", UrXLearning)

void UrXLearning::switch_target()
{
  std::map<std::string, std::vector<double>> jointTarget;
  switch(phase_)
  {
    case IDLE:
      postureTask->target(defaultPosture);
      phase_ = MOVE_UP;
      break;

    case MOVE_UP:
      jointTarget[moveJoint] = {jointAngDefault + moveMag};
      postureTask->target(jointTarget);
      gripperPostureTask_->target(gripperClose);
      phase_ = MOVE_DOWN;
      break;

    case MOVE_DOWN:
      jointTarget[moveJoint] = {jointAngDefault - moveMag};
      postureTask->target(jointTarget);
      gripperPostureTask_->target(gripperOpen);
      phase_ = MOVE_UP;
      break;
  }
}

void UrXLearning::store_target()
{
  // std::map<std::string, std::vector<double>> jointTarget;
  switch(store_state_)
  {
    case START:
      postureTask->target(storePoseStart);
      store_state_ = TURN;
      break;

    case TURN:
      postureTask->target(storePoseTurn);
      break;
  }
}