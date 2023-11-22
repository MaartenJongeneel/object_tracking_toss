#include "TerminateProcess.h"

#include "../ObjectTrackingToss.h"

void TerminateProcess::configure(const mc_rtc::Configuration & config) {}

void TerminateProcess::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<ObjectTrackingToss &>(ctl_);
}

bool TerminateProcess::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<ObjectTrackingToss &>(ctl_);
  
  auto io = mc_iam::IO::get(ctl.robot("ur10_with_sr_gripper"));
  io->stop_optitrack();
  io->stop_blowoff();
  io->stop_vacuum();
  
  output("OK");
  return true;
}

void TerminateProcess::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<ObjectTrackingToss &>(ctl_);
  
  mc_rtc::log::success("Congratulations: Process Completed, the process is now going to die :)");
  exit(EXIT_SUCCESS);
}

EXPORT_SINGLE_STATE("TerminateProcess", TerminateProcess)
