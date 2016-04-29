#include <ros/ros.h>

#include <Stargazer.hpp>

int main(int argc, char** argv)
{
    // Initialize ROS
    ros::init(argc, argv, "srsdrv_stargazer");

    ROS_INFO_STREAM("srsdrv_stargazer_node started");

    // Create the stargazer driver and run it
    srs::Stargazer stargazer;
    stargazer.run();

    return 0;
}
