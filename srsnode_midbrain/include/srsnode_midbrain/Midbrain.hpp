/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <srslib_framework/ros/unit/RosUnit.hpp>

#include <srslib_framework/ros/function/RosTriggerShutdown.hpp>
#include <srsnode_midbrain/Reflexes.hpp>

namespace srs {

class Midbrain : public RosUnit<Midbrain>
{
public:
    Midbrain(string name, int argc, char** argv);

    ~Midbrain()
    {}

protected:
    void execute();

    void initialize();

private:
    void evaluateTriggers();

    constexpr static double REFRESH_RATE_HZ = 50;

    Reflexes reflexes_;

    RosTriggerShutdown triggerShutdown_;
};

} // namespace srs
