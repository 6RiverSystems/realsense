/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <algorithm>
using namespace std;

#include <sensor_msgs/Joy.h>

#include <srslib_framework/robotics/Velocity.hpp>
#include <srslib_framework/ros/tap/subscriber/RosSubscriber.hpp>
#include <srslib_framework/ros/topics/ChuckTopics.hpp>

namespace srs {

class TapJoy :
    public RosSubscriber<sensor_msgs::Joy>
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

    TapJoy() :
        RosSubscriber(ChuckTopics::sensor::JOYSTICK_RAW, 10, "~"),
        joystickLeft_(Velocity<>::INVALID),
        joystickRight_(Velocity<>::INVALID)
    {
        fill(currentButtons_, currentButtons_ + MAX_NUMBER_BUTTONS, 0);
        fill(previousButtons_, previousButtons_ + MAX_NUMBER_BUTTONS, 0);
    }

    ~TapJoy()
    {}

    bool anyButtonChanged()
    {
        for (unsigned int b = 0; b < MAX_NUMBER_BUTTONS; b++)
        {
            if (isButtonChanged(static_cast<ButtonEnum>(b)))
            {
                return true;
            }
        }

        return false;
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

    bool isButtonChanged(ButtonEnum button)
    {
        return isButtonPressed(button) || isButtonReleased(button);
    }

    bool isButtonPressed(ButtonEnum button)
    {
        declareStale();

        return (button >= 0 && button < MAX_NUMBER_BUTTONS) ?
            currentButtons_[button] && !previousButtons_[button] : false;
    }

    bool isButtonReleased(ButtonEnum button)
    {
        declareStale();

        return (button >= 0 && button < MAX_NUMBER_BUTTONS) ?
            !currentButtons_[button] && previousButtons_[button] : false;
    }

    Velocity<> peekJoystickLeft() const
    {
        return joystickLeft_;
    }

    Velocity<> peekJoystickRight() const
    {
        return joystickRight_;
    }

    Velocity<> popJoystickLeft()
    {
        declareStale();
        return joystickLeft_;
    }

    Velocity<> popJoystickRight()
    {
        declareStale();
        return joystickRight_;
    }

    void receiveData(const sensor_msgs::Joy::ConstPtr message)
    {
        // Extract the current velocity from the left and right axis
        set(Velocity<>(message->axes[1], message->axes[0]),
            Velocity<>(message->axes[3], message->axes[2]));

        // Copy the previous state of the buttons
        copy(currentButtons_, currentButtons_ + MAX_NUMBER_BUTTONS, previousButtons_);

        // Update the current state of the buttons
        for (unsigned int b = 0; b < message->buttons.size(); b++)
        {
            currentButtons_[b] = message->buttons[b] > 0;
        }
    }

    void reset()
    {
        RosSubscriber::reset();

        joystickLeft_ = Velocity<>::INVALID;
        joystickRight_ = Velocity<>::INVALID;

        fill(currentButtons_, currentButtons_ + MAX_NUMBER_BUTTONS, 0);
        fill(previousButtons_, previousButtons_ + MAX_NUMBER_BUTTONS, 0);
    }

    virtual void set(Velocity<> joystickLeft, Velocity<> joystickRight)
    {
        RosSubscriber::set();

        joystickLeft_ = joystickLeft;
        joystickRight_ = joystickRight;
    }

private:
    constexpr static unsigned int MAX_NUMBER_BUTTONS = 20;

    bool currentButtons_[MAX_NUMBER_BUTTONS];

    Velocity<> joystickLeft_;
    Velocity<> joystickRight_;

    bool previousButtons_[MAX_NUMBER_BUTTONS];
};

} // namespace srs
