#include "ObjectTrackingToss.h"

#include <mc_iam/devices/IO.h>

ObjectTrackingToss::ObjectTrackingToss(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
: mc_control::fsm::Controller(rm, dt, config)
{

  mc_rtc::log::success("ObjectTrackingToss init done ");
}

bool ObjectTrackingToss::run()
{
  return mc_control::fsm::Controller::run();
}

void ObjectTrackingToss::reset(const mc_control::ControllerResetData & reset_data)
{
  mc_control::fsm::Controller::reset(reset_data);
  
  auto io = mc_iam::IO::get(robot());
  if(!io)
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("[URBasicTossSceneSRGripper] {} has no I.AM IO attached", robot().name());
  }
  auto gui = this->gui();
  if(!gui)
  {
    return;
  }
  gui->addElement({"AGX"}, mc_rtc::gui::Robot("box", [this]() -> const mc_rbdyn::Robot & { return realRobots().robot("box"); }));
}
