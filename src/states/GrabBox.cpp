#include "GrabBox.h"
#include "../ObjectTrackingToss.h"

#include<iostream>
#include<sstream>
#include<fstream>
#include<iomanip>

void GrabBox::configure(const mc_rtc::Configuration & config)
{
  state_conf_.load(config);
  if(state_conf_.has("box_position_hover")){
    box_position_hover_ = state_conf_("box_position_hover");
    mc_rtc::log::success("in configure method loading the config, found box_position_hover");
    mc_rtc::log::info("box_position_hover_ {}", box_position_hover_);
  }

  if(state_conf_.has("box_position_pick")){
    box_position_pick_ = state_conf_("box_position_pick");
    mc_rtc::log::success("in configure method loading the config, found box_position_pick");
    mc_rtc::log::info("box_position_pick_ {}", box_position_pick_);
  }

  if(state_conf_.has("box_position_lift")){
    box_position_lift_ = state_conf_("box_position_lift");
    mc_rtc::log::success("in configure method loading the config, found box_position_lift");
    mc_rtc::log::info("box_position_lift_ {}", box_position_lift_);
  }

}

void GrabBox::start(mc_control::fsm::Controller & ctl_)
{
    auto & ctl = static_cast<ObjectTrackingToss &>(ctl_);
    
    ctl.robot().mbc().q = ctl.realRobot().mbc().q;
    ctl.robot().mbc().alpha = ctl.realRobot().mbc().alpha;
    
    auto io = mc_iam::IO::get(ctl.robot("ur10_with_sr_gripper"));
    io->stop_blowoff();
    io->stop_vacuum();
    io->start_optitrack();
    
    sleep(5);

    ctl.efTask_ = std::make_shared<mc_tasks::EndEffectorTask>("suction_cup_link", ctl.robots(), 0, 10, 1e6);
    ctl.efTask_->orientationTask->weight(5e6);
    ctl.solver().addTask(ctl.efTask_);

        auto target = sva::PTransformd{box_position_hover_}; //Current Box 007 picking 0.46769, -0.85083, 1.1, Box 5 picking: 0.43269, -0.87083, 1.1
        target.rotation() = sva::RotY(M_PI);
        ctl.efTask_->set_ef_pose(target);
    
}

bool GrabBox::run(mc_control::fsm::Controller & ctl_) // Called every iteration until it returns true
{
    auto & ctl = static_cast<ObjectTrackingToss &>(ctl_);
    
    auto io = mc_iam::IO::get(ctl.robot("ur10_with_sr_gripper"));
    
    if(!finishedUp && ctl.efTask_->eval().norm() > 0.01) {
        return false;
    }
    else if(!finishedUp) {
        finishedUp = true;
            auto target = sva::PTransformd{box_position_pick_};
            target.rotation() = sva::RotY(M_PI);
            ctl.efTask_->set_ef_pose(target);
            io->start_vacuum();
        return false;
    }
    else if(ctl.efTask_->eval().norm() > 0.01) {
        return false;
    }
    else if(finishedUp && !finishedUp2) {
        finishedUp2 = true;
            auto target = sva::PTransformd{box_position_lift_};
            target.rotation() = sva::RotY(M_PI);
            ctl.efTask_->set_ef_pose(target);
        return false;
    }
    else if(ctl.efTask_->eval().norm() > 0.4) {
        return false;
    }
//    else if(finishedUp2 && !finishedUp3) {
//        finishedUp3 = true;
//            auto target = sva::PTransformd{Eigen::Vector3d(0.40, -0.50,1.1)};
//            target.rotation() = sva::RotY(M_PI);
//            ctl.efTask_->set_ef_pose(target);
//        return false;
//    }
//    else if(ctl.efTask_->eval().norm() > 0.001) {
//        return false;
//    }
    else {
        io->start_optitrack();

        
        output("OK");
        return true;
    }
}

void GrabBox::teardown(mc_control::fsm::Controller & ctl_)
{
    auto & ctl = static_cast<ObjectTrackingToss &>(ctl_);
    ctl_.solver().removeTask(ctl.efTask_);
}

EXPORT_SINGLE_STATE("GrabBox", GrabBox)
