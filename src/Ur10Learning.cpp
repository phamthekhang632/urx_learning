#include "Ur10Learning.h"

Ur10Learning::Ur10Learning(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
: mc_control::MCController(rm, dt)
{
  config_.load(config);

  solver().addConstraintSet(contactConstraint);
  solver().addConstraintSet(selfCollisionConstraint);
  solver().addConstraintSet(dynamicsConstraint);
  solver().addConstraintSet(compoundJointConstraint);
  solver().addTask(postureTask);
  postureTask->stiffness(1);
  postureTask->weight(1);

  moveTask_ = mc_tasks::MetaTaskLoader::load<mc_tasks::EndEffectorTask>(
    solver(), config("MoveTask"));

  mc_rtc::log::success("Ur10Learning init done ");
}

bool Ur10Learning::run()
{
  if (phase_ == IDLE)
  {
    postureTask->target
    ({
      {"shoulder_pan_joint", {0.0}}, 
      {"shoulder_lift_joint", {-M_PI * 5/6}},
      {"elbow_joint", {M_PI * 2/3}}
    });

    moveTask_->selectUnactiveJoints(solver(), joints_);

    if (postureTask->eval().norm() < 0.1 && postureTask->speed().norm() < 0.1)
    {
      phase_ = MOVE_BACKWARD;
      moveTask_->selectActiveJoints(solver(), joints_);
    }
  }
  else if (phase_ == MOVE_BACKWARD 
    && moveTask_->eval().norm() < 0.01 && moveTask_->speed().norm() < 0.1)
  {
    phase_ = MOVE_FORWARD;
    moveTask_->add_ef_pose({Eigen::Vector3d(-0.2, 0.0, 0.0)});
  }
  else if (phase_ == MOVE_FORWARD
    && moveTask_->eval().norm() < 0.01 && moveTask_->speed().norm() < 0.1)
  {
    phase_ = MOVE_BACKWARD;
    moveTask_->add_ef_pose({Eigen::Vector3d(0.2, 0.0, 0.0)});
  }
  return mc_control::MCController::run();
}

void Ur10Learning::reset(const mc_control::ControllerResetData & reset_data)
{
  moveTask_->reset();
  solver().addTask(moveTask_);

  mc_control::MCController::reset(reset_data);
}

CONTROLLER_CONSTRUCTOR("Ur10Learning", Ur10Learning)
