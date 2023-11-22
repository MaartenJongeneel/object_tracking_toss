#include "MoveToNeutral.h"
#include "../ObjectTrackingToss.h"

void MoveToNeutral::configure(const mc_rtc::Configuration & config)
{
}

void MoveToNeutral::start(mc_control::fsm::Controller & ctl_)
{
    auto & ctl = static_cast<ObjectTrackingToss &>(ctl_);
    
    ctl.robot().mbc().q = ctl.realRobot().mbc().q;
    ctl.robot().mbc().alpha = ctl.realRobot().mbc().alpha;

    ctl.efTask_ = std::make_shared<mc_tasks::EndEffectorTask>("suction_cup_link", ctl.robots(), 0, 10, 1e6);
    ctl.efTask_->orientationTask->weight(5e6);
    ctl.solver().addTask(ctl.efTask_);
    auto target = sva::PTransformd{Eigen::Vector3d(0.585,-0.4,1.1)};
    target.rotation() = sva::RotY(M_PI);//*sva::RotZ(-0.5*M_PI);
    ctl.efTask_->set_ef_pose(target);
}

bool MoveToNeutral::run(mc_control::fsm::Controller & ctl_) // Called every iteration until it returns true
{
    auto & ctl = static_cast<ObjectTrackingToss &>(ctl_);
    
    auto io = mc_iam::IO::get(ctl.robot("ur10_with_sr_gripper"));

    if(!finishedUp && ctl.efTask_->eval().norm() > 0.0001) {
        return false;
    }
    else {
        io->conveyor_stop();
        
        output("OK");
        return true;
    }    
}

void MoveToNeutral::teardown(mc_control::fsm::Controller & ctl_)
{
    auto & ctl = static_cast<ObjectTrackingToss &>(ctl_);

    ctl_.solver().removeTask(ctl.efTask_);
}

EXPORT_SINGLE_STATE("MoveToNeutral", MoveToNeutral)
