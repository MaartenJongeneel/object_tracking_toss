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

class ViveTracking {
private:

public:
    vr::IVRSystem* vr_pointer {NULL}; // Calls IVRSystem constructor, vr_pointer can be assigned with functions from the constructor.
    vr::HmdVector3_t GetPosition(vr::HmdMatrix34_t matrix);
    vr::HmdQuaternion_t GetRotation(vr::HmdMatrix34_t matrix);

    ~ViveTracking(); // Destructor
    ViveTracking(); // Constructor

    // Button events
    bool ProcessVREvent(const vr::VREvent_t & event);
    void dealWithButtonEvent(vr::VREvent_t event);
    bool teleOpClutch {};
    bool startTeleOp {true};
    bool triggerPressed {};
    bool safetyClutch {};
    bool blowoffSwitch {};

    // Retrieve pose data from controller
    std::vector<double> &getAbsPoseData(std::vector<double> &q_track);

    // Declare data variables
    vr::VRControllerState_t state;
    vr::TrackedDevicePose_t trackedControllerPose;
    vr::VRControllerState_t controllerState;
    vr::HmdMatrix34_t viveTrackData;
    vr::HmdVector3_t pos;
    vr::HmdQuaternion_t quater;
    double scaling {2.5};

    struct _ControllerData {
        // Fields to be initialzed by iterateAssignIds() and setHands()
        int deviceId = -1;   // Device ID according to the SteamVR system
        int hand = -1;       // 0=invalid 1=left 2=right
        int idtrigger = -1;  // Trigger axis id
        int idpad = -1;      // Touchpad axis id
        //Position set in ControllerPose()
        vr::HmdVector3_t pos{};
        bool isValid{};
    }; typedef struct _ControllerData ControllerData;
    // An array of ControllerData structs
    ControllerData controllers[2];
};

struct RecordRobotTeleoperation : mc_control::fsm::State {

    void configure(const mc_rtc::Configuration & config) override;
    void start(mc_control::fsm::Controller & ctl) override;
    bool run(mc_control::fsm::Controller & ctl) override;
    void teardown(mc_control::fsm::Controller & ctl) override;

    ViveTracking viveTracking;

    sva::PTransformd efPoseMatrix;
    sva::PTransformd efPoseStartMatrix;

    std::vector<double> viveTrackData;

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

    mc_rtc::Logger logger = {mc_rtc::Logger::Policy::THREADED, "/home/administrator/devel/controllers/erf-teleoperation-controller/tossing_recordings", "teleoperation"};
};
