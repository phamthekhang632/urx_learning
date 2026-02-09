#include "Ur10Learning.h"

Ur10Learning::Ur10Learning(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
: mc_control::MCController(rm, dt)
{
  solver().addConstraintSet(contactConstraint);
  solver().addConstraintSet(selfCollisionConstraint);
  solver().addConstraintSet(dynamicsConstraint);
  solver().addConstraintSet(compoundJointConstraint);
  postureTask->stiffness(0.5);
  postureTask->weight(1);
  solver().addTask(postureTask);

  mc_rtc::log::success("Ur10Learning init done ");
}

bool Ur10Learning::run()
{  
  if(postureTask->eval().norm() < 0.02 && postureTask->speed().norm() < 0.01) {
    switch_target();
  }
  return mc_control::MCController::run();
}

void Ur10Learning::reset(const mc_control::ControllerResetData & reset_data)
{
  mc_control::MCController::reset(reset_data);
}

void Ur10Learning::switch_target()
{
  std::map<std::string, std::vector<double>> jointTarget;
  switch (phase_)
  {
    case IDLE:
      postureTask->target
      ({
        {"shoulder_pan_joint", {M_PI / 2}}, 
        {"shoulder_lift_joint", {-M_PI / 2}},
        {"elbow_joint", {M_PI / 2}},
        {"wrist_1_joint", {jointAngDefault}}
      });
      phase_ = MOVE_UP;
      break;

    case MOVE_UP:
      jointTarget[moveJoint] = {jointAngDefault + moveMag};
      postureTask->target(jointTarget);
      phase_ = MOVE_DOWN;
      break;

    case MOVE_DOWN:
      jointTarget[moveJoint] = {jointAngDefault - moveMag};
      postureTask->target(jointTarget);
      phase_ = MOVE_UP;
      break;
  }
}

CONTROLLER_CONSTRUCTOR("Ur10Learning", Ur10Learning)
