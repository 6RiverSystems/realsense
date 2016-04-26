#include <ros/ros.h>

#include <srsnode_joystick_adapter/JoystickAdapter.hpp>

int main(int argc, char** argv)
{
    // Initialize ROS and ROSCPP
    ros::init(argc, argv, "node_pe");
    ros::NodeHandle nh;

    ROS_INFO_STREAM("joystick_adapter_node started");

    // Create the estimator and run it
    srs::JoystickAdapter joystickAdapter;
    joystickAdapter.run();

    return 0;
}
