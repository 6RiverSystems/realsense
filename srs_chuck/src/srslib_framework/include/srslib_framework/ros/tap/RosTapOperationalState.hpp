/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef ROSTAPOPERATIONSTATE_HPP_
#define ROSTAPOPERATIONSTATE_HPP_

#include <algorithm>
using namespace std;

#include <srslib_framework/MsgOperationalState.h>

#include <srslib_framework/robotics/RobotState.hpp>
#include <srslib_framework/ros/tap/RosTap.hpp>
#include <srslib_framework/ros/message/OperationalStateMessageFactory.hpp>

namespace srs {

class RosTapOperationalState :
    public RosTap
{
public:
    RosTapOperationalState() :
        RosTap("/info/operational_state", "Operational State Tap"),
        currentRobotState_()
    {}

    ~RosTapOperationalState()
    {
        disconnectTap();
    }

    bool isPauseChanged()
    {
        setNewData(false);
        return currentRobotState_.pause != previousRobotState_.pause;
    }

    bool isStateChanged()
    {
        setNewData(false);
        return currentRobotState_.any() != previousRobotState_.any();
    }

    bool getPause()
    {
        setNewData(false);
        return currentRobotState_.pause;
    }

    RobotState getRobotState()
    {
        setNewData(false);
        return currentRobotState_;
    }

    void reset()
    {
        RosTap::reset();

        previousRobotState_ = RobotState();
        currentRobotState_ = previousRobotState_;
    }

protected:
    bool connect()
    {
        rosSubscriber_ = rosNodeHandle_.subscribe(getTopic(), 10,
            &RosTapOperationalState::onOperationalState, this);
        return true;
    }

private:
    void onOperationalState(const srslib_framework::MsgOperationalStateConstPtr& message)
    {
        previousRobotState_ = currentRobotState_;
        currentRobotState_ = OperationalStateMessageFactory::msg2RobotState(message);

        setNewData(true);
    }

    RobotState currentRobotState_;
    RobotState previousRobotState_;
};

} // namespace srs

#endif // ROSTAPOPERATIONSTATE_HPP_
