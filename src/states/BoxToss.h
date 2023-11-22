#pragma once

#include <mc_control/fsm/State.h>
#include <mc_tasks/EndEffectorTask.h>
#include <iostream>
#include <headers/openvr.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
// #include <mc_iam/devices/Vacuum.h>
#include <mc_iam/devices/IO.h>
#include <mc_rtc/log/Logger.h>
#include <mc_rbdyn/rpy_utils.h>
#include <mc_rtc/log/FlatLog.h>
#include <mc_tasks/MetaTaskLoader.h>

struct BoxToss : mc_control::fsm::State
{
    void configure(const mc_rtc::Configuration & config) override;
    void start(mc_control::fsm::Controller & ctl) override;
    bool run(mc_control::fsm::Controller & ctl) override;
    void teardown(mc_control::fsm::Controller & ctl) override;

    mc_rtc::Configuration state_conf_;

    sva::PTransformd efPoseMatrix;
    sva::PTransformd efPoseStartMatrix;
    
    std::vector<double> viveTrackData;

    std::string toss_file_name_ = "";

    double startViveTrackPosX;
    double startViveTrackPosY;
    double startViveTrackPosZ;
    Eigen::Matrix3d startViveTrackOri {};

    double relativeViveTrackPosX;
    double relativeViveTrackPosY;
    double relativeViveTrackPosZ;
    Eigen::Matrix3d relativeViveTrackOri {};

    double refInputPosX;
    double refInputPosY;
    double refInputPosZ;
    Eigen::Matrix3d refInputOri {};

    bool vacuumSwitch {};
    bool teleOpClutch {};
    bool startTeleOp {true};
    bool triggerPressed {};
    bool safetyClutch {};
    bool blowoffSwitch {};
    bool runningConveyor {};
    
    int i = 0;

private:
};
