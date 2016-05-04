/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef REALSENSE_DRIVER_H_
#define REALSENSE_DRIVER_H_

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

using namespace std;

namespace srs {

class RealsenseDriver
{
public:
    RealsenseDriver();

    ~RealsenseDriver()
    {

    }

    void run();

private:

    void onInfrared1( const sensor_msgs::Image& infraredImage1 );

    void onInfrared2( const sensor_msgs::Image& infraredImage2 );

    void onPointCloud( const sensor_msgs::PointCloud2& pointCloud );

private:

    constexpr static unsigned int REFRESH_RATE_HZ = 50;

    ros::NodeHandle rosNodeHandle_;

    ros::Subscriber infrared1Subscriber_;

    ros::Subscriber infrared2Subscriber_;

    ros::Subscriber pointCloudSubscriber_;
};

} // namespace srs

#endif // REALSENSE_DRIVER_H_
