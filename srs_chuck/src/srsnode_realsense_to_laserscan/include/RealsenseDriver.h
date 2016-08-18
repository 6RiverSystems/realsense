/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef REALSENSE_DRIVER_H_
#define REALSENSE_DRIVER_H_

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <message_filters/subscriber.h>
#include <cv_bridge/cv_bridge.h>

using namespace std;

namespace srs {

class RealsenseDriver
{

public:
    RealsenseDriver();

    ~RealsenseDriver() { };

    void run();

private:

    void OnDepthData( const sensor_msgs::Image::ConstPtr& infraredImage );

private:

    cv_bridge::CvImagePtr GetCvImage( const sensor_msgs::Image::ConstPtr& image ) const;

    constexpr static unsigned int REFRESH_RATE_HZ = 50;

    ros::NodeHandle 									rosNodeHandle_;

    ros::Subscriber						 				depthSubscriber_;

    ros::Publisher 										depthMedianFilterPublisher_;

};

} // namespace srs

#endif // REALSENSE_DRIVER_H_
