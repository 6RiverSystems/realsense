#include <srsnode_joystick_adapter/JoystickAdapter.hpp>

#include <cmath>
using namespace std;

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include <dynamic_reconfigure/server.h>

#include <srslib_framework/math/BasicMath.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
JoystickAdapter::JoystickAdapter(string nodeName) :
    joystickLatched_(false),
    rosNodeHandle_(nodeName)
{
    pubCommand_ = rosNodeHandle_.advertise<geometry_msgs::Twist>(
        "/internal/sensors/joystick/velocity", 50);
    pubJoystickLatched_ = rosNodeHandle_.advertise<std_msgs::Bool>(
        "/internal/sensors/joystick/latched", 1);
    pubJoystickEmergency_ = rosNodeHandle_.advertise<std_msgs::Bool>(
        "/internal/sensors/joystick/emergency", 1);

    configServer_.setCallback(boost::bind(&JoystickAdapter::onConfigChange, this, _1, _2));
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void JoystickAdapter::run()
{
    tapJoy_.connectTap();
    triggerShutdown_.connectService();

    ros::Rate refreshRate(REFRESH_RATE_HZ);
    while (ros::ok())
    {
        ros::spinOnce();

        evaluateTriggers();

        if (tapJoy_.newDataAvailable())
        {
            // If the B button was pressed, publish the emergency state immediately
            if (tapJoy_.isButtonPressed(RosTapJoy<>::BUTTON_B))
            {
                publishEmergency(true);
                ROS_WARN("The joystick emergency button has been pressed");
            }
            else if (tapJoy_.isButtonReleased(RosTapJoy<>::BUTTON_B))
            {
                publishEmergency(false);
                ROS_WARN("The joystick emergency button has been released");
            }

            if (tapJoy_.isButtonChanged(RosTapJoy<>::BUTTON_START))
            {
                ROS_INFO_COND(joystickLatched_, "The joystick latch has been removed");

                // Remove the latch if the START button has been pressed
                joystickLatched_ = false;
            }
            else if (!joystickLatched_)
            {
                ROS_INFO("The joystick latch has been established");
                joystickLatched_ = true;
            }
            publishLatched(joystickLatched_);

            // Update the velocity values
            Velocity<> currentVelocity = tapJoy_.getVelocity();

            double linear = configuration_.ratio_linear * currentVelocity.linear;
            double angular = configuration_.ratio_angular * currentVelocity.angular;

            linear = BasicMath::threshold<double>(linear, configuration_.threshold_linear, 0.0);
            angular = BasicMath::threshold<double>(angular, configuration_.threshold_angular, 0.0);

            publishVelocity(Velocity<>(linear, angular));
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
void JoystickAdapter::onConfigChange(JoystickConfig& config, uint32_t level)
{
    configuration_ = config;

    ROS_INFO_STREAM("Joystick configuration changed: (" <<
        configuration_.ratio_linear << ", " << configuration_.ratio_angular <<
        configuration_.threshold_linear << ", " << configuration_.threshold_angular << ")");
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void JoystickAdapter::publishEmergency(bool state)
{
    std_msgs::Bool messageEmergency;

    messageEmergency.data = state;
    pubJoystickEmergency_.publish(messageEmergency);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void JoystickAdapter::publishLatched(bool state)
{
    std_msgs::Bool messageLatched;

    messageLatched.data = state;
    pubJoystickLatched_.publish(messageLatched);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void JoystickAdapter::publishVelocity(Velocity<> velocity)
{
    geometry_msgs::Twist messageVelocity;

    messageVelocity.linear.x = velocity.linear;
    messageVelocity.linear.y = 0.0;
    messageVelocity.linear.z = 0.0;

    messageVelocity.angular.x = 0.0;
    messageVelocity.angular.y = 0.0;
    messageVelocity.angular.z = velocity.angular;

    pubCommand_.publish(messageVelocity);
}

} // namespace srs
