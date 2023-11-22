#pragma once

#include <mc_control/fsm/State.h>
#include <mc_tasks/SurfaceTransformTask.h>
#include <mc_tasks/EndEffectorTask.h>
#include <mc_iam/devices/IO.h>
#include<iostream>
#include<sstream>
#include<fstream>
#include<iomanip>
#include<unistd.h>
#include<cstdlib>

struct GrabBox : mc_control::fsm::State
{
    mc_rtc::Configuration state_conf_;

    void configure(const mc_rtc::Configuration & config) override;
    void start(mc_control::fsm::Controller & ctl) override;
    bool run(mc_control::fsm::Controller & ctl) override;
    void teardown(mc_control::fsm::Controller & ctl) override;   

    Eigen::Vector3d box_position_hover_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d box_position_pick_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d box_position_lift_ = Eigen::Vector3d::Zero();
    
private:
    std::shared_ptr<mc_tasks::SurfaceTransformTask> surfTransTask_;
    std::string robot_;

    bool finishedUp{false};
    bool finishedUp2{false};
    bool finishedUp3{false};
};
