/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef ROSTAPJOY_HPP_
#define ROSTAPJOY_HPP_

#include <string>
#include <algorithm>
using namespace std;

#include <sensor_msgs/Joy.h>

#include <srslib_framework/ros/RosTap.hpp>
#include <srslib_framework/robotics/Velocity.hpp>

namespace srs {

template<typename TYPE = double>
class RosTapJoy :
    public RosTap
{
public:
    // TODO Rename the buttons with PS3 names
    enum ButtonEnum {
        BUTTON_FIRE = 0,
        BUTTON_2 = 1,
        BUTTON_3 = 2,
        BUTTON_4 = 3,
        BUTTON_5 = 4,
        BUTTON_6 = 5,
        BUTTON_7 = 6,
        BUTTON_8 = 7,
        BUTTON_9 = 8,
        BUTTON_10 = 9,
        BUTTON_11 = 10
    };

    RosTapJoy() :
        RosTap("/joy", "Joy Tap"),
        currentVelocity_()
    {
        fill(currentButtons_, currentButtons_ + NUMBER_BUTTONS, 0);
        fill(previousButtons_, previousButtons_ + NUMBER_BUTTONS, 0);
    }

    ~RosTapJoy()
    {
        disconnectTap();
    }

    bool anyButtonPressed()
    {
        for (unsigned int b = 0; b < NUMBER_BUTTONS; b++)
        {
            if (isButtonPressed(static_cast<ButtonEnum>(b)))
            {
                return true;
            }
        }

        return false;
    }

    bool anyButtonReleased()
    {
        for (unsigned int b = 0; b < NUMBER_BUTTONS; b++)
        {
            if (isButtonReleased(static_cast<ButtonEnum>(b)))
            {
                return true;
            }
        }

        return false;
    }

    Velocity<TYPE> getVelocity()
    {
        setNewData(false);
        return currentVelocity_;
    }

    bool isButtonPressed(ButtonEnum button)
    {
        setNewData(false);
        return (button >= 0 && button < NUMBER_BUTTONS) ?
            currentButtons_[button] && !previousButtons_[button] : false;
    }

    bool isButtonReleased(ButtonEnum button)
    {
        setNewData(false);
        return (button >= 0 && button < NUMBER_BUTTONS) ?
            !currentButtons_[button] && previousButtons_[button] : false;
    }

protected:
    bool connect()
    {
        rosSubscriber_ = rosNodeHandle_.subscribe(getTopic(), 10, &RosTapJoy::onJoy, this);
        return true;
    }

private:
    constexpr static unsigned int NUMBER_BUTTONS = 11;

    bool currentButtons_[NUMBER_BUTTONS];
    Velocity<TYPE> currentVelocity_;

    bool previousButtons_[NUMBER_BUTTONS];

    void onJoy(const sensor_msgs::Joy::ConstPtr& message)
    {
        currentVelocity_ = Velocity<TYPE>(message->axes[1], message->axes[0]);

        copy(currentButtons_, currentButtons_ + NUMBER_BUTTONS, previousButtons_);
        for (unsigned int b = 0; b < NUMBER_BUTTONS; b++)
        {
            currentButtons_[b] = message->buttons[b] > 0;
        }

        setNewData(true);
    }
};

} // namespace srs

#endif // ROSTAPJOY_HPP_
