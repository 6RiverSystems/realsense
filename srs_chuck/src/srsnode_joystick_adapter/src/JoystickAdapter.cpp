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
    rosNodeHandle_(nodeName),
    tapJoy_(),
    triggerShutdown_(rosNodeHandle_)
{
    pubCommand_ = rosNodeHandle_.advertise<geometry_msgs::Twist>("velocity", 50);
    pubJoystickLatched_ = rosNodeHandle_.advertise<std_msgs::Bool>("latched", 1);

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
            if (!tapJoy_.anyButtonPressed() && !tapJoy_.anyButtonReleased())
            {
                if (!joystickLatched_)
                {
                    joystickLatched_ = true;
                    ROS_INFO("The joystick latch has been established");
                }

                Velocity<> currentVelocity = tapJoy_.getVelocity();

                double linear = configuration_.ratio_linear * currentVelocity.linear;
                double angular = configuration_.ratio_angular * currentVelocity.angular;

                geometry_msgs::Twist messageVelocity;

                messageVelocity.linear.x = BasicMath::threshold<double>(linear,
                    configuration_.threshold_linear, 0.0);
                messageVelocity.linear.y = 0.0;
                messageVelocity.linear.z = 0.0;

                messageVelocity.angular.x = 0.0;
                messageVelocity.angular.y = 0.0;
                messageVelocity.angular.z = BasicMath::threshold<double>(angular,
                    configuration_.threshold_angular, 0.0);

                pubCommand_.publish(messageVelocity);
            }
            else if (tapJoy_.isButtonPressed(RosTapJoy<>::BUTTON_11) && joystickLatched_)
            {
                joystickLatched_ = false;
                ROS_INFO("The joystick latch has been removed");
            }

            std_msgs::Bool messageLatched;
            messageLatched.data = joystickLatched_;

            pubJoystickLatched_.publish(messageLatched);
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

} // namespace srs
