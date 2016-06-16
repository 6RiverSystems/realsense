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
    enum ButtonEnum {
        BUTTON_A = 0,
        BUTTON_B = 1,
        BUTTON_2 = 2, // Not present in the PS2 joypad
        BUTTON_X = 3,
        BUTTON_Y = 4,
        BUTTON_5 = 5, // Not present int the PS2 joypad
        BUTTON_LEFT_TOP = 6,
        BUTTON_RIGHT_TOP = 7,
        BUTTON_LEFT_BOTTOM = 8,
        BUTTON_RIGHT_BOTTOM = 9,
        BUTTON_SELECT = 10,
        BUTTON_START = 11,
        BUTTON_MODE = 12
    };

    RosTapJoy() :
        RosTap("/joy", "Joy Tap"),
        currentVelocity_()
    {
        fill(currentButtons_, currentButtons_ + MAX_NUMBER_BUTTONS, 0);
        fill(previousButtons_, previousButtons_ + MAX_NUMBER_BUTTONS, 0);
    }

    ~RosTapJoy()
    {
        disconnectTap();
    }

    bool anyButtonPressed()
    {
        for (unsigned int b = 0; b < MAX_NUMBER_BUTTONS; b++)
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
        for (unsigned int b = 0; b < MAX_NUMBER_BUTTONS; b++)
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

    bool isButtonChanged(ButtonEnum button)
    {
        return isButtonPressed(button) || isButtonReleased(button);
    }

    bool isButtonPressed(ButtonEnum button)
    {
        setNewData(false);
        return (button >= 0 && button < MAX_NUMBER_BUTTONS) ?
            currentButtons_[button] && !previousButtons_[button] : false;
    }

    bool isButtonReleased(ButtonEnum button)
    {
        setNewData(false);
        return (button >= 0 && button < MAX_NUMBER_BUTTONS) ?
            !currentButtons_[button] && previousButtons_[button] : false;
    }

    void reset()
    {
        RosTap::reset();

        currentVelocity_ = Velocity<>();
        fill(currentButtons_, currentButtons_ + MAX_NUMBER_BUTTONS, 0);
        fill(previousButtons_, previousButtons_ + MAX_NUMBER_BUTTONS, 0);
    }

protected:
    bool connect()
    {
        rosSubscriber_ = rosNodeHandle_.subscribe(getTopic(), 10, &RosTapJoy::onJoy, this);
        return true;
    }

private:
    constexpr static unsigned int MAX_NUMBER_BUTTONS = 20;

    void onJoy(const sensor_msgs::Joy::ConstPtr& message)
    {
        // Extract the current velocity from the axis
        currentVelocity_ = Velocity<TYPE>(message->axes[1], message->axes[0]);

        // Copy the previous state of the buttons
        copy(currentButtons_, currentButtons_ + MAX_NUMBER_BUTTONS, previousButtons_);

        // Update the current state of the buttons
        for (unsigned int b = 0; b < message->buttons.size(); b++)
        {
            currentButtons_[b] = message->buttons[b] > 0;
        }

        setNewData(true);
    }

    bool currentButtons_[MAX_NUMBER_BUTTONS];
    Velocity<TYPE> currentVelocity_;

    bool previousButtons_[MAX_NUMBER_BUTTONS];
};

} // namespace srs

#endif // ROSTAPJOY_HPP_
