#include "ObjectTrackingToss_Initial.h"

#include "../ObjectTrackingToss.h"

void ObjectTrackingToss_Initial::configure(const mc_rtc::Configuration & config) {}

void ObjectTrackingToss_Initial::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<ObjectTrackingToss &>(ctl_);
}

bool ObjectTrackingToss_Initial::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<ObjectTrackingToss &>(ctl_);
  output("OK");
  return true;
}

void ObjectTrackingToss_Initial::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<ObjectTrackingToss &>(ctl_);
}

EXPORT_SINGLE_STATE("ObjectTrackingToss_Initial", ObjectTrackingToss_Initial)
