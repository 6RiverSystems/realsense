/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef REALSENSE_DRIVER_H_
#define REALSENSE_DRIVER_H_

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <cv_bridge/cv_bridge.h>

using namespace std;

namespace srs {

class RealsenseDriver
{
	typedef message_filters::TimeSynchronizer<sensor_msgs::LaserScan, sensor_msgs::Image, sensor_msgs::Image> ImageSyncronizer;

	typedef boost::shared_ptr<ImageSyncronizer> ImageSyncronizerPtr;

public:
    RealsenseDriver();

    ~RealsenseDriver()
    {

    }

    void run();

private:

    void OnDepthData( const sensor_msgs::LaserScan::ConstPtr& scan, const sensor_msgs::Image::ConstPtr& infraredImage1,
    	const sensor_msgs::Image::ConstPtr& infraredImage2 );

private:

    cv_bridge::CvImagePtr GetCvImage( const sensor_msgs::Image::ConstPtr& image ) const;

    void ThresholdImage( cv::Mat& image ) const;

    void CombineImages( cv::Mat& image1, cv::Mat& image2, cv::Mat& result ) const;

    constexpr static unsigned int REFRESH_RATE_HZ = 50;

    ros::NodeHandle 									rosNodeHandle_;

    message_filters::Subscriber<sensor_msgs::Image> 	infrared1Subscriber_;

    message_filters::Subscriber<sensor_msgs::Image> 	infrared2Subscriber_;

    message_filters::Subscriber<sensor_msgs::LaserScan> laserScanSubscriber_;

    ros::Publisher 										infrared1Publisher_;

    ros::Publisher 										infrared2Publisher_;

    ros::Publisher 										infraredPublisher_;

    ImageSyncronizerPtr									synchronizer_;

};

} // namespace srs

#endif // REALSENSE_DRIVER_H_
