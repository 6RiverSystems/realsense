#include <srsnode_joystick_adapter/JoystickAdapter.hpp>

#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
JoystickAdapter::JoystickAdapter(string nodeName) :
    joystickLatched_(false),
    rosNodeHandle_(nodeName),
    tapJoy_(rosNodeHandle_),
    triggerShutdown_(rosNodeHandle_)
{
    pubCommand_ = rosNodeHandle_.advertise<geometry_msgs::Twist>("velocity", 50);
    pubJoystickLatched_ = rosNodeHandle_.advertise<std_msgs::Bool>("latched", 1);
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

                double linear = currentVelocity.linear * RATIO_LINEAR;
                double angular = sign(linear) * currentVelocity.angular * RATIO_ANGULAR;

                geometry_msgs::Twist messageVelocity;
                messageVelocity.linear.x = abs(linear) > THRESHOLD_LINEAR ? linear : 0.0;
                messageVelocity.linear.y = 0.0;
                messageVelocity.linear.z = 0.0;

                messageVelocity.angular.x = 0.0;
                messageVelocity.angular.y = 0.0;
                messageVelocity.angular.z = abs(angular) > THRESHOLD_ANGULAR ? angular : 0.0;

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
    if (triggerShutdown_.isShutdownRequested())
    {
        ros::shutdown();
    }
}

} // namespace srs
