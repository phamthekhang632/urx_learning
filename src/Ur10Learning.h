#pragma once

#include <mc_control/mc_controller.h>


#include "api.h"

struct Ur10Learning_DLLAPI Ur10Learning : public mc_control::MCController
{
  Ur10Learning(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config);

  bool run() override;

  void reset(const mc_control::ControllerResetData & reset_data) override;
};