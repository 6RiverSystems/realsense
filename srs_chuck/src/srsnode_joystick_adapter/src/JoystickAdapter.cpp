#include <srsnode_joystick_adapter/JoystickAdapter.hpp>

#include <cmath>
using namespace std;

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include <dynamic_reconfigure/server.h>

#include <srslib_framework/math/BasicMath.hpp>
#include <srslib_framework/math/VelocityMath.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
JoystickAdapter::JoystickAdapter(string nodeName) :
    rosNodeHandle_(nodeName),
    publisherJoypadState_("/internal/sensors/joystick/state", 50)
{
    configServer_.setCallback(boost::bind(&JoystickAdapter::onConfigChange, this, _1, _2));
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void JoystickAdapter::run()
{
    triggerShutdown_.connectService();

    ros::Rate refreshRate(REFRESH_RATE_HZ);
    while (ros::ok())
    {
        ros::spinOnce();

        evaluateTriggers();

        if (tapJoy_.newDataAvailable())
        {
            // Management of the B button: Emergency
            if (tapJoy_.isButtonPressed(RosTapJoy::BUTTON_B))
            {
                joypadState_.buttonEmergency = true;
                ROS_WARN("The joystick emergency button has been pressed");
            }
            else if (tapJoy_.isButtonReleased(RosTapJoy::BUTTON_B))
            {
                joypadState_.buttonEmergency = false;
                ROS_WARN("The joystick emergency button has been released");
            }

            // Management of the X button: Generic
            if (tapJoy_.isButtonPressed(RosTapJoy::BUTTON_X))
            {
                joypadState_.buttonX = true;
                ROS_WARN("The joystick X button has been pressed");
            }
            else if (tapJoy_.isButtonReleased(RosTapJoy::BUTTON_X))
            {
                joypadState_.buttonX = false;
                ROS_WARN("The joystick X button has been released");
            }

            // Management of the Y button: Generic
            if (tapJoy_.isButtonPressed(RosTapJoy::BUTTON_Y))
            {
                joypadState_.buttonY = true;
                ROS_WARN("The joystick Y button has been pressed");
            }
            else if (tapJoy_.isButtonReleased(RosTapJoy::BUTTON_Y))
            {
                joypadState_.buttonY = false;
                ROS_WARN("The joystick Y button has been released");
            }

            // Management of the A button: Custom action
            if (tapJoy_.isButtonPressed(RosTapJoy::BUTTON_A))
            {
                joypadState_.buttonAction = true;
                ROS_INFO("The joystick custom action button has been pressed");
            }
            else if (tapJoy_.isButtonReleased(RosTapJoy::BUTTON_A))
            {
                joypadState_.buttonAction = false;
                ROS_INFO("The joystick custom action button has been released");
            }

            if (tapJoy_.isButtonPressed(RosTapJoy::BUTTON_START))
            {
                // If the START button has been released
                // there is no need to do anything else
                joypadState_.latched = false;
                ROS_INFO("The joystick latch has been removed");
            }

            // Update the velocity values
            Velocity<> leftVelocity = tapJoy_.popJoystickLeft();
            Velocity<> rightVelocity = tapJoy_.popJoystickRight();

            if (!VelocityMath::equal(leftVelocity, currentLeftVelocity_) ||
                !VelocityMath::equal(rightVelocity, currentRightVelocity_))
            {
                ROS_INFO_COND(!joypadState_.latched, "The joystick latch has been established");
                joypadState_.latched = true;
            }

            currentLeftVelocity_ = leftVelocity;
            currentRightVelocity_ = rightVelocity;

            double linear = configuration_.ratio_linear * leftVelocity.linear;
            double angular = configuration_.ratio_angular * rightVelocity.angular;

            linear = BasicMath::threshold<double>(linear, configuration_.threshold_linear, 0.0);
            angular = BasicMath::threshold<double>(angular, configuration_.threshold_angular, 0.0);

            joypadState_.velocity = Velocity<>(linear, angular);

            joypadState_.arrivalTime = ros::Time::now().toSec();
            publisherJoypadState_.publish(joypadState_);
        }

        refreshRate.sleep();
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods

////////////////////////////////////////////////////////////////////////////////////////////////////
void JoystickAdapter::evaluateTriggers()
{
    if (triggerShutdown_.isTriggerRequested())
    {
        ros::shutdown();
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void JoystickAdapter::onConfigChange(srsnode_joystick_adapter::JoystickConfig& config, uint32_t level)
{
    configuration_ = config;

    ROS_INFO_STREAM("Joystick configuration changed: (" <<
        configuration_.ratio_linear << ", " << configuration_.ratio_angular <<
        configuration_.threshold_linear << ", " << configuration_.threshold_angular << ")");
}

} // namespace srs
