#include <ros/ros.h>

#include <srsnode_joystick_adapter/JoystickAdapter.hpp>

int main(int argc, char** argv)
{
    // Initialize ROS
    ros::init(argc, argv, "srsnode_joystick_adapter");

    ROS_INFO_STREAM("srsnode_joystick_adapter started");

    // Create the estimator and run it
    srs::JoystickAdapter joystickAdapter;
    joystickAdapter.run();

    return 0;
}
