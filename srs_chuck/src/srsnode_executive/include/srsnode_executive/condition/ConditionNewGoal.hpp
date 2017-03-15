/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <geometry_msgs/PoseStamped.h>

#include <srslib_framework/platform/SoftwareMessageHandler.hpp>
#include <srslib_framework/robotics/Pose.hpp>
#include <srslib_framework/ros/tap/TapMoveBaseCurrentGoal.hpp>
#include <srslib_framework/ros/tap/subscriber/Subscriber.hpp>

#include <srsnode_executive/ExecutiveContext.hpp>

namespace srs {

class Executive;

class ConditionNewGoal :
    public SoftwareMessageHandler<Executive, geometry_msgs::PoseStamped>
{
public:
    ConditionNewGoal(Executive* owner);

    virtual ~ConditionNewGoal()
    {}

    virtual void notified(Subscriber<geometry_msgs::PoseStamped>* subject);

    void evaluate(ExecutiveContext& context);

private:
    Pose<> currentGoal_;

    bool newGoalReceived_;

    TapMoveBaseCurrentGoal tapCurrentGoal_;
};

} // namespace srs
