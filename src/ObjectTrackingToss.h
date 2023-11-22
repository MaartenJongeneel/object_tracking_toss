#pragma once

#include <mc_control/fsm/Controller.h>

#include "api.h"

struct ObjectTrackingToss_DLLAPI ObjectTrackingToss : public mc_control::fsm::Controller
{
  ObjectTrackingToss(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config);

  bool run() override;

  void reset(const mc_control::ControllerResetData & reset_data) override;
  
  std::shared_ptr<mc_tasks::EndEffectorTask> efTask_;
  std::shared_ptr<mc_solver::DynamicsConstraint> boxDynamics;

private:
  mc_rtc::Configuration config_;
    std::vector<double> jointTorquesAGX_;
    Eigen::VectorXd jointTorquesAGX;
    Eigen::VectorXd jointTorquesMCRTC;
};
