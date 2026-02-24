#pragma once

#include <mc_control/mc_controller.h>

#include "api.h"
enum ControlState
{
  IDLE = 0,
  MOVE_UP,
  MOVE_DOWN
};

struct Ur10Learning_DLLAPI Ur10Learning : public mc_control::MCController
{
  Ur10Learning(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config);
  bool run() override;
  void reset(const mc_control::ControllerResetData & reset_data) override;
  
  private:
    void switch_target();

    ControlState phase_ = IDLE;
    std::string moveJoint = "wrist_1_joint";
    double jointAngDefault = 0;
    double moveMag = M_PI / 6;
};