#include "Ur10Learning.h"

Ur10Learning::Ur10Learning(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
: mc_control::MCController(rm, dt)
{
  solver().addConstraintSet(contactConstraint);
  solver().addConstraintSet(selfCollisionConstraint);
  solver().addConstraintSet(dynamicsConstraint);
  solver().addConstraintSet(compoundJointConstraint);
  postureTask->stiffness(1);
  postureTask->weight(1);
  solver().addTask(postureTask);

  mc_rtc::log::success("Ur10Learning init done ");
}

bool Ur10Learning::run()
{
    postureTask->target
    ({
      {"shoulder_pan_joint", {M_PI / 2}}, 
      {"shoulder_lift_joint", {-M_PI / 2}},
      {"elbow_joint", {M_PI / 2}}
    });
  return mc_control::MCController::run();
}

void Ur10Learning::reset(const mc_control::ControllerResetData & reset_data)
{
  mc_control::MCController::reset(reset_data);
}

CONTROLLER_CONSTRUCTOR("Ur10Learning", Ur10Learning)
