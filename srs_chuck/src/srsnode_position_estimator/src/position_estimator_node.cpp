#include <string>
using namespace std;

#include <ros/ros.h>

#include <srsnode_position_estimator/PositionEstimator.hpp>

int main(int argc, char** argv)
{
    const static string NODE_NAME = "srsnode_position_estimator";

    // Initialize ROS
    ros::init(argc, argv, NODE_NAME);
    ROS_INFO_STREAM(NODE_NAME << " started");

    // Create the estimator and run it
    srs::PositionEstimator positionEstimator(NODE_NAME);
    positionEstimator.run();

    return 0;
}
