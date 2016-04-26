#include <srsnode_joystick_adapter/JoystickAdapter.hpp>

#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_broadcaster.h>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
JoystickAdapter::JoystickAdapter() :
    rosNodeHandle_()
{
    rosPubCmdVel_ = rosNodeHandle_.advertise<nav_msgs::Odometry>("/cmd_vel", 50);
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

            messageCmdVel.linear.x = currentVelocity.linear;
            messageCmdVel.linear.y = 0.0;
            messageCmdVel.linear.z = 0.0;

            messageCmdVel.angular.x = 0.0;
            messageCmdVel.angular.y = 0.0;
            messageCmdVel.angular.z = currentVelocity.angular;

            rosPubCmdVel_.publish(messageCmdVel);
        }

        refreshRate.sleep();
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods

} // namespace srs
