#pragma once

#include <mc_control/fsm/State.h>
#include <mc_tasks/SurfaceTransformTask.h>
#include <mc_tasks/EndEffectorTask.h>
#include <mc_iam/devices/IO.h>

struct MoveToStart : mc_control::fsm::State
{
    void configure(const mc_rtc::Configuration & config) override;
    void start(mc_control::fsm::Controller & ctl) override;
    bool run(mc_control::fsm::Controller & ctl) override;
    void teardown(mc_control::fsm::Controller & ctl) override;
private:
    std::shared_ptr<mc_tasks::SurfaceTransformTask> surfTransTask_;
    std::string robot_;
    bool finishedUp{false};
    bool finishedUp2{false};
};
