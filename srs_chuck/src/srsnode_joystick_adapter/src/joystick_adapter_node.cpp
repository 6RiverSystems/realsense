#include <ros/ros.h>

#include <srsnode_joystick_adapter/JoystickAdapter.hpp>

int main(int argc, char** argv)
{
    const static string NODE_NAME = "srsnode_joystick_adapter";

    // Initialize ROS
    ros::init(argc, argv, NODE_NAME);
    ROS_INFO_STREAM(NODE_NAME << " started");

    // Create the estimator and run it
    srs::JoystickAdapter joystickAdapter(NODE_NAME);
    joystickAdapter.run();

    return 0;
}
