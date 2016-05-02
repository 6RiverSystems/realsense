#include <srsnode_joystick_adapter/JoystickAdapter.hpp>

#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
JoystickAdapter::JoystickAdapter() :
    rosNodeHandle_()
{
    rosPubCmdVel_ = rosNodeHandle_.advertise<geometry_msgs::Twist>("/cmd_vel", 50);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void JoystickAdapter::run()
{
    tapJoy_.connectTap();

    ros::Rate refreshRate(REFRESH_RATE_HZ);
    while (ros::ok())
    {
        ros::spinOnce();

        if (tapJoy_.newDataAvailable())
        {
            Velocity<> currentVelocity = tapJoy_.getCurrentVelocity();

            geometry_msgs::Twist messageCmdVel;

            double linear = currentVelocity.linear * RATIO_LINEAR;
            double angular = currentVelocity.angular * RATIO_ANGULAR;

            messageCmdVel.linear.x = abs(linear) > THRESHOLD_LINEAR ? linear : 0.0;
            messageCmdVel.linear.y = 0.0;
            messageCmdVel.linear.z = 0.0;

            messageCmdVel.angular.x = 0.0;
            messageCmdVel.angular.y = 0.0;
            messageCmdVel.angular.z = abs(angular) > THRESHOLD_ANGULAR ? angular : 0.0;

            rosPubCmdVel_.publish(messageCmdVel);
        }

        refreshRate.sleep();
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods

} // namespace srs
