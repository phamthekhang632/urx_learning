#include "UrXLearning.h"

#include <mc_rbdyn/RobotLoader.h>
#include <chrono>
#include <thread>

UrXLearning::UrXLearning(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
: mc_control::MCController({rm, mc_rbdyn::RobotLoader::get_robot_module("robotiq_arg85"),
                            mc_rbdyn::RobotLoader::get_robot_module("env/ground")},
                           dt)
{
  solver().addConstraintSet(contactConstraint);
  solver().addConstraintSet(selfCollisionConstraint);
  postureTask->setGains(0.5, 1.4);
  postureTask->weight(1);
  solver().addTask(postureTask);
  solver().setContacts({{}});

  mc_rtc::log::success("UrXLearning init done ");
}

bool UrXLearning::run()
{
  if(postureTask->eval().norm() < 0.02 && postureTask->speed().norm() < 0.01)
  {
    if(false)
      switch_target();
    else
      store_target();
  }
  return mc_control::MCController::run();
}

void UrXLearning::reset(const mc_control::ControllerResetData & reset_data)
{
  mc_control::MCController::reset(reset_data);
  // robot(1) = robotiq_arg85
  robots().robot(1).posW(sva::PTransformd(sva::RotX(-M_PI / 2), Eigen::Vector3d(0.8172, 0.2329, 0.0628)));
  addContact({"ur5e", "robotiq_arg85", "Tool", "Base"});

  gripperPostureTask_ = std::make_shared<mc_tasks::PostureTask>(solver(), 1);
  solver().addTask(gripperPostureTask_);
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