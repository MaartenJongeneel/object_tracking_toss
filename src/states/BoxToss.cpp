#include "BoxToss.h"

#include "../ObjectTrackingToss.h"

void BoxToss::configure(const mc_rtc::Configuration & config)
{
  state_conf_.load(config);
  if(state_conf_.has("toss_file_name")){
    toss_file_name_ = "";
    toss_file_name_.append(state_conf_("toss_file_name"));
    mc_rtc::log::success("in configure method loading the config, found toss_file_name");
    mc_rtc::log::info("toss_file_name_ {}", toss_file_name_);
  }
}

void BoxToss::start(mc_control::fsm::Controller & ctl_)
{
    auto & ctl = static_cast<ObjectTrackingToss &>(ctl_);
    
    ctl.robot().mbc().q = ctl.realRobot().mbc().q;
    ctl.robot().mbc().alpha = ctl.realRobot().mbc().alpha;

    ctl.efTask_ = std::make_shared<mc_tasks::EndEffectorTask>("suction_cup_link", ctl.robots(), 0, 65, 1e6);
    ctl.efTask_->orientationTask->weight(100e3);
    ctl.solver().addTask(ctl.efTask_); 

    ctl.robot().mbc().q = ctl.realRobot().mbc().q;
    ctl.robot().mbc().alpha = ctl.realRobot().mbc().alpha;
}

bool BoxToss::run(mc_control::fsm::Controller & ctl_)
{
    auto & ctl = static_cast<ObjectTrackingToss &>(ctl_);
    
    mc_rtc::log::FlatLog flat(toss_file_name_);
    
    if(flat.size() == i) {
        output("OK");
        return true;
    } else {
    
    auto viveTrackData_i = flat.getRaw<std::vector<double>>("viveTrackData", i);
    
    auto vacuumSwitch_i = flat.getRaw<bool>("vacuumSwitch", i);
    auto teleOpClutch_i = flat.getRaw<bool>("teleOpClutch", i);
    auto startTeleOp_i = flat.getRaw<bool>("startTeleOp", i);
    auto triggerPressed_i = flat.getRaw<bool>("triggerPressed", i);
    auto safetyClutch_i = flat.getRaw<bool>("safetyClutch", i);
    auto blowoffSwitch_i = flat.getRaw<bool>("blowoffSwitch", i);
    
    viveTrackData = *viveTrackData_i;
    vacuumSwitch = *vacuumSwitch_i;
    teleOpClutch = *teleOpClutch_i;
    startTeleOp = *startTeleOp_i;
    triggerPressed = *triggerPressed_i;
    safetyClutch = *safetyClutch_i;
    blowoffSwitch = *blowoffSwitch_i;
    
        // Convert tracked orientation in quaternions to rotation matrix
        Eigen::Quaterniond quaternion(viveTrackData[3], viveTrackData[4], -viveTrackData[6], -viveTrackData[5]);
        Eigen::Matrix3d viveOrientationMatrix = quaternion.normalized().toRotationMatrix();

        // Monitor actual pose of end-effector
        efPoseMatrix = ctl.robot().bodyPosW("suction_cup_link");
    
        // Pose control
        if(safetyClutch) {
        // If touchpad pressed:
            if(teleOpClutch) {
                if (startTeleOp) {
                    // Vive Tracking
                    startViveTrackPosX = viveTrackData[0];
                    startViveTrackPosY = viveTrackData[2];
                    startViveTrackPosZ = viveTrackData[1];
                    startViveTrackOri = viveOrientationMatrix;
                    // EF tracking
                    efPoseStartMatrix = efPoseMatrix;
                    // Only do the above things at first iteration
                    startTeleOp = false;
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
                // If touchpad is NOT pressed, use last end-effector pose as input pose (robot will stop moving immediately)
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
        if(safetyClutch) {
            if (triggerPressed && !vacuumSwitch) {
                io->stop_blowoff();
                io->start_vacuum();
                mc_rtc::log::success("Vacuum suction gripper activated");
                vacuumSwitch = true; // Vacuum only has to be activated on the first iteration that (teleOpClutch && triggerPressed)
            } else if(!triggerPressed && vacuumSwitch) {
                io->stop_vacuum();
                io->start_blowoff();
                mc_rtc::log::success("Vacuum suction gripper deactivated");
                mc_rtc::log::success("Blow-off activated");
                vacuumSwitch = false; // Vacuum only has to be deactivated on the first iteration that (!teleOpClutch && !triggerPressed)
            }
        }
        if(blowoffSwitch) {
            io->stop_blowoff();
            io->stop_vacuum();
            mc_rtc::log::success("Blow-off and vacuum deactivated");
            blowoffSwitch = false;
        }
    }
    
    i++;

    return false;
}

void BoxToss::teardown(mc_control::fsm::Controller & ctl_)
{
    auto & ctl = static_cast<ObjectTrackingToss &>(ctl_);
    
    if(runningConveyor) {
        auto io = mc_iam::IO::get(ctl.robot("ur10_with_sr_gripper"));
        io->conveyor_stop();
    }

    auto io = mc_iam::IO::get(ctl.robot("ur10_with_sr_gripper"));
    io->stop_blowoff();
    io->stop_optitrack();
    
    ctl_.solver().removeTask(ctl.efTask_);
    
}

EXPORT_SINGLE_STATE("BoxToss", BoxToss)
