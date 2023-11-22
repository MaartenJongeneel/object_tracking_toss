#include "RecordRobotTeleoperation.h"
#include "../ObjectTrackingToss.h"

// Destructor for the PoseTracking object
ViveTracking::~ViveTracking() {
    if (vr_pointer != NULL) {
        // VR Shutdown: https://github.com/ValveSoftware/openvr/wiki/API-Documentation#initialization-and-cleanup
        vr::VR_Shutdown();
        vr_pointer = NULL;
    }
}

// Constructor for the PoseTracking object
ViveTracking::ViveTracking() {
    vr::EVRInitError eError {vr::VRInitError_None}; // eError holds class member VRInitError_None from enum EVRInitError, which provides the errors in the system.
    vr_pointer = VR_Init(&eError, vr::VRApplication_Background); // Initializes the system, and returns a vr::IVRSystem pointer that allows to call other OpenVR API methods.
    if (eError != vr::VRInitError_None) { // Checks if there are errors.
        vr_pointer = NULL;
        mc_rtc::log::error("Unable to init VR runtime: {}",VR_GetVRInitErrorAsEnglishDescription(eError));
        exit(EXIT_FAILURE);
    }
}

bool ViveTracking::ProcessVREvent(const vr::VREvent_t & event) {
    switch(event.eventType) {
        case vr::VREvent_Quit:
            mc_rtc::log::error("Received SteamVR Quit: {}",vr::VREvent_Quit);
            return false;
            break;
        case vr::VREvent_ProcessQuit:
            mc_rtc::log::error("SteamVR Quit Process: {}",vr::VREvent_ProcessQuit);
            return false;
            break;
        case vr::VREvent_QuitAcknowledged:
            mc_rtc::log::error("SteamVR Quit Acknowledged: {}",vr::VREvent_QuitAcknowledged);
            return false;
            break;
        case vr::VREvent_TrackedDeviceActivated:
            mc_rtc::log::info("Device #{}: Activated",event.trackedDeviceIndex);
            break;
        case vr::VREvent_TrackedDeviceDeactivated:
            mc_rtc::log::info("Device #{}: Deactivated",event.trackedDeviceIndex);
            break;
        case vr::VREvent_TrackedDeviceUpdated:
            mc_rtc::log::info("Device #{}: Updated",event.trackedDeviceIndex);
            break;
        default:
            if(event.eventType >= 200 && event.eventType <= 203) // Button events range from 200-203
                dealWithButtonEvent(event);
    }
    return true;
}

void ViveTracking::dealWithButtonEvent(vr::VREvent_t event)
{
    int controllerIndex; // The index of the controllers[] array that corresponds with the controller that had a buttonEvent
    for (int i = 0; i < 2; i++) // Iterates across the array of controllers
    {
        ControllerData* pController = &(controllers[i]);
        if(pController->deviceId == event.trackedDeviceIndex) // This tests to see if the current controller from the loop is the same from the event
            controllerIndex = i;
    }
    switch(event.eventType) {
        case vr::VREvent_ButtonPress:
            switch(event.data.controller.button)
            {
                case vr::k_EButton_SteamVR_Trigger:
                    triggerPressed = true;
                    break;
                case vr::k_EButton_SteamVR_Touchpad:
                    if (safetyClutch) {
                        mc_rtc::log::success("Teleoperation started");
                    }
                    teleOpClutch = true;
                    startTeleOp = true;
                    break;
                case vr::k_EButton_Grip:
                    mc_rtc::log::success("Data transmission resumed");
                    safetyClutch = true;
                    break;
                case vr::k_EButton_ApplicationMenu:
                    mc_rtc::log::success("Blow-off switched off");
                    blowoffSwitch = true;
                    break;
            }
            break;
        case vr::VREvent_ButtonUnpress:
            switch(event.data.controller.button)
            {
                case vr::k_EButton_SteamVR_Trigger:
                    triggerPressed = false;
                    break;
                case vr::k_EButton_SteamVR_Touchpad:
                    if (safetyClutch) {
                        mc_rtc::log::success("Teleoperation stopped");
                    }
                    teleOpClutch = false;
                    break;
                case vr::k_EButton_Grip:
                    mc_rtc::log::warning("Data transmission stopped");
                    safetyClutch = false;
                    startTeleOp = true;
                    break;
            }
            break;
    }
}

vr::HmdQuaternion_t ViveTracking::GetRotation(vr::HmdMatrix34_t matrix) {
    vr::HmdQuaternion_t q;
    q.w = sqrt(fmax(0, 1 + matrix.m[0][0] + matrix.m[1][1] + matrix.m[2][2])) / 2;
    q.x = sqrt(fmax(0, 1 + matrix.m[0][0] - matrix.m[1][1] - matrix.m[2][2])) / 2;
    q.y = sqrt(fmax(0, 1 - matrix.m[0][0] + matrix.m[1][1] - matrix.m[2][2])) / 2;
    q.z = sqrt(fmax(0, 1 - matrix.m[0][0] - matrix.m[1][1] + matrix.m[2][2])) / 2;
    q.x = copysign(q.x, matrix.m[2][1] - matrix.m[1][2]);
    q.y = copysign(q.y, matrix.m[0][2] - matrix.m[2][0]);
    q.z = copysign(q.z, matrix.m[1][0] - matrix.m[0][1]);
    return q;
}

std::vector<double> &ViveTracking::getAbsPoseData(std::vector<double> &q_track) {
    for (vr::TrackedDeviceIndex_t unDevice = 0; unDevice < vr::k_unMaxTrackedDeviceCount; unDevice++) {
        if (!vr_pointer->IsTrackedDeviceConnected(unDevice))
            continue;

        if (vr_pointer->GetControllerState(unDevice, &state, sizeof(state))) {
            vr::ETrackedDeviceClass trackedDeviceClass = vr_pointer->GetTrackedDeviceClass(unDevice);
            if(trackedDeviceClass==vr::ETrackedDeviceClass::TrackedDeviceClass_Controller) {
                vr_pointer->GetControllerStateWithPose(vr::TrackingUniverseStanding, unDevice,
                                                       &controllerState, sizeof(controllerState), &trackedControllerPose);
                
                viveTrackData = trackedControllerPose.mDeviceToAbsoluteTracking;
                pos.v[0] = scaling*viveTrackData.m[0][3];
                pos.v[1] = scaling*viveTrackData.m[1][3];
                pos.v[2] = scaling*viveTrackData.m[2][3];
                quater = GetRotation(trackedControllerPose.mDeviceToAbsoluteTracking);
//                q_track = {viveTrackData.m[0][3], viveTrackData.m[1][3], viveTrackData.m[2][3], quater.w, quater.x, quater.y, quater.z};
                q_track = {pos.v[0], pos.v[1], pos.v[2], quater.w, quater.x, quater.y, quater.z};
                // std::cout << "x: " << pos.v[0] << "   y: " << pos.v[1] << "   z: " << pos.v[2] << "\n";
            }
        }
    }
    return q_track;
}

void RecordRobotTeleoperation::configure(const mc_rtc::Configuration & config) {
}

void RecordRobotTeleoperation::start(mc_control::fsm::Controller & ctl_) {
    auto & ctl = static_cast<ObjectTrackingToss &>(ctl_);

    ctl.robot().mbc().q = ctl.realRobot().mbc().q;
    ctl.robot().mbc().alpha = ctl.realRobot().mbc().alpha;

    ctl.efTask_ = std::make_shared<mc_tasks::EndEffectorTask>("suction_cup_link", ctl.robots(), 0, 45, 1e6);
    ctl.efTask_->orientationTask->weight(100e3);
    ctl.solver().addTask(ctl.efTask_);
    mc_rtc::log::info("To use the controller, always make sure there is a controller device detected by SteamVR");
    mc_rtc::log::info("Press grip in order to start data transmission");
    
    logger.addLogEntry("vacuumSwitch", [this]() { return vacuumSwitch; });
    logger.addLogEntry("teleOpClutch", [this]() { return viveTracking.teleOpClutch; });
    logger.addLogEntry("startTeleOp", [this]() { return viveTracking.startTeleOp; });
    logger.addLogEntry("triggerPressed", [this]() { return viveTracking.triggerPressed; });
    logger.addLogEntry("safetyClutch", [this]() { return viveTracking.safetyClutch; });
    logger.addLogEntry("blowoffSwitch", [this]() { return viveTracking.blowoffSwitch; });
    
    logger.addLogEntry("viveTrackData", [this]() { return viveTrackData; });

    logger.start("ur10", ctl.solver().dt());
}

bool RecordRobotTeleoperation::run(mc_control::fsm::Controller & ctl_) {
    auto &ctl = static_cast<ObjectTrackingToss &>(ctl_);
    
    vr::VREvent_t event;
    if(viveTracking.vr_pointer->PollNextEvent(&event, sizeof(event))) // See ProcessVREvent below, returns false if the function determines the type of error to be fatal or signal some kind of quit.
    {
        if (!viveTracking.ProcessVREvent(event)) {
            mc_rtc::log::error("OpenVR service quit");
            return false;
        }
    }

    // Get tracking data: viveTrackData = [x, y, z, q.w, q.x, q.y, q.z]
    viveTrackData = viveTracking.getAbsPoseData(viveTrackData);
    
    logger.log();
    
    // Convert tracked orientation in quaternions to rotation matrix
    Eigen::Quaterniond quaternion(viveTrackData[3], viveTrackData[4], -viveTrackData[6], -viveTrackData[5]);
    Eigen::Matrix3d viveOrientationMatrix = quaternion.normalized().toRotationMatrix();

    // Monitor actual pose of end-effector
    efPoseMatrix = ctl.robot().bodyPosW("suction_cup_link");
//    efPoseMatrix = ctl.realRobot().bodyPosW("suction_cup_link");
    
    // Pose control
    if(viveTracking.safetyClutch) {
        // If touchpad pressed:
        if(viveTracking.teleOpClutch) {
            if (viveTracking.startTeleOp) {
                // Vive Tracking
                startViveTrackPosX = viveTrackData[0];
                startViveTrackPosY = viveTrackData[2];
                startViveTrackPosZ = viveTrackData[1];
                startViveTrackOri = viveOrientationMatrix;
                // EF tracking
                efPoseStartMatrix = efPoseMatrix;
                // Only do the above things at first iteration
                viveTracking.startTeleOp = false;
            }
            // Pose tracking of Vive relative to starting point of teleoperation
            relativeViveTrackPosX = viveTrackData[0] - startViveTrackPosX;
            relativeViveTrackPosY = viveTrackData[2] - startViveTrackPosY;
            relativeViveTrackPosZ = viveTrackData[1] - startViveTrackPosZ;
            relativeViveTrackOri = startViveTrackOri.inverse() * viveOrientationMatrix; // Subtract start orientation from current orientation.
            // Determine sum of end-effector pose with relative vive pose change while teleoperation is "on" and set as reference input:
            refInputPosX = efPoseStartMatrix.translation()[0] - relativeViveTrackPosX; // Add relative Vive position w.r.t start teleoperation to end-effector starting position.
            refInputPosY = efPoseStartMatrix.translation()[1] + relativeViveTrackPosY;
            refInputPosZ = efPoseStartMatrix.translation()[2] + relativeViveTrackPosZ;
            // Add relative Vive orientation w.r.t start teleoperation to end-effector starting orientation.
            refInputOri = efPoseStartMatrix.rotation()*relativeViveTrackOri;

            // Set end-effector task with reference input:
            auto target = sva::PTransformd{Eigen::Vector3d(refInputPosX, refInputPosY, refInputPosZ)};
            target.rotation() = refInputOri;
            ctl.efTask_->set_ef_pose(target);

        } else {
            // If touchpadio->start_optitrack(); is NOT pressed, use last end-effector pose as input pose (robot will stop moving immediately)
            refInputPosX = efPoseMatrix.translation()[0];
            refInputPosY = efPoseMatrix.translation()[1];
            refInputPosZ = efPoseMatrix.translation()[2];
            refInputOri = efPoseMatrix.rotation();

            // Set end-effector task with reference input:
            auto target = sva::PTransformd{Eigen::Vector3d(refInputPosX, refInputPosY, refInputPosZ)};
            target.rotation() = refInputOri;
            ctl.efTask_->set_ef_pose(target);
        }
    }
    
    // Vacuum gripper control
    auto io = mc_iam::IO::get(ctl.robot("ur10_with_sr_gripper"));
    if(viveTracking.safetyClutch) {
        if (viveTracking.triggerPressed && !vacuumSwitch) {
            io->stop_blowoff();
            io->start_vacuum();
            mc_rtc::log::success("Vacuum suction gripper activated");
            vacuumSwitch = true; // Vacuum only has to be activated on the first iteration that (teleOpClutch && triggerPressed)
        } else if(!viveTracking.triggerPressed && vacuumSwitch) {
            io->stop_vacuum();
            io->start_blowoff();
            mc_rtc::log::success("Vacuum suction gripper deactivated");
            mc_rtc::log::success("Blow-off activated");
            vacuumSwitch = false; // Vacuum only has to be deactivated on the first iteration that (!teleOpClutch && !triggerPressed)
        } else if(viveTracking.blowoffSwitch) {
            io->stop_blowoff();
            io->stop_vacuum();
            mc_rtc::log::success("Blow-off and vacuum deactivated");
            viveTracking.blowoffSwitch = false;
        }
    }
    
    if(viveTracking.blowoffSwitch && viveTracking.triggerPressed && viveTracking.teleOpClutch) {
        output("OK");
        return true;
    }

    return false;
}

void RecordRobotTeleoperation::teardown(mc_control::fsm::Controller & ctl_) {
    auto & ctl = static_cast<ObjectTrackingToss &>(ctl_);
    
    ctl_.solver().removeTask(ctl.efTask_);
    
    ctl.logger().removeLogEntries(this);
}

EXPORT_SINGLE_STATE("RecordRobotTeleoperation", RecordRobotTeleoperation)
