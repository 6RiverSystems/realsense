#include "ImageSubscriber.hpp"

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/opencv.hpp>
//#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/features2d.hpp>

const std::string ImageSubscriber::WINDOW_TITLE = "OHPS 1";

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
ImageSubscriber::ImageSubscriber() :
    imageTransport_(rosNodeHandle_)
{
    imageSubscriber_ = imageTransport_.subscribe("/camera/image_raw", 1, &ImageSubscriber::cbImageReceived, this);
    cv::namedWindow(WINDOW_TITLE);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
ImageSubscriber::~ImageSubscriber()
{
    cv::destroyWindow(WINDOW_TITLE);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void ImageSubscriber::cbImageReceived(const sensor_msgs::ImageConstPtr& message)
{
    cv_bridge::CvImagePtr receivedImage;

    try
    {
        receivedImage = cv_bridge::toCvCopy(message, sensor_msgs::image_encodings::RGB8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat decoratedImage(receivedImage->image);

    std::vector<cv::KeyPoint> keypoints;
    poseEstimator_.estimate(receivedImage->image, keypoints);

    unsigned int xSize = receivedImage->image.cols;
    unsigned int ySize = receivedImage->image.rows;

    cv::line(decoratedImage, cv::Point(0, 430), cv::Point(xSize, 430), CV_RGB(0, 0, 255), 2, 0, 0);
    cv::line(decoratedImage, cv::Point(0, 760), cv::Point(xSize, 760), CV_RGB(0, 0, 255), 2, 0, 0);

//    cv::line(decoratedImage, cv::Point(xSize / 2, 0), cv::Point(xSize / 2, ySize), CV_RGB(0, 255, 0));
//    cv::line(decoratedImage, cv::Point(0, ySize / 2), cv::Point(xSize, ySize / 2), CV_RGB(0, 255, 0));

    cv::drawKeypoints(decoratedImage, keypoints, decoratedImage,
            cv::Scalar(0, 0, 255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

    cv::circle(decoratedImage, cv::Point(50, 50), 10, CV_RGB(255,0,0));

    // Update GUI Window
    cv::imshow(WINDOW_TITLE, decoratedImage);
    cv::waitKey(1);
}
