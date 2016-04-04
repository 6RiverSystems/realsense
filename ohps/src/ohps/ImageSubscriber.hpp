#ifndef _IMAGESUBSCRIBER_HPP_
#define _IMAGESUBSCRIBER_HPP_

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "PoseEstimator.hpp"

class ImageSubscriber
{
public:
    ImageSubscriber();
    ~ImageSubscriber();

private:
    static const std::string WINDOW_TITLE;

    void cbImageReceived(const sensor_msgs::ImageConstPtr& message);

    ros::NodeHandle rosNodeHandle_;
    image_transport::ImageTransport imageTransport_;
    image_transport::Subscriber imageSubscriber_;

    PoseEstimator poseEstimator_;
};

#endif  // _IMAGESUBSCRIBER_HPP_
