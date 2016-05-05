#include <ros/ros.h>

#include <StarGazerDriver.hpp>

int main(int argc, char** argv)
{
    // Initialize ROS
    ros::init(argc, argv, "srsdrv_stargazer");

    ROS_INFO_STREAM("srsdrv_stargazer_node started");

    // Create the stargazer driver and run it
    srs::StarGazerDriver stargazer;

    stargazer.run();

    return 0;
}
