#include "PoseEstimator.hpp"

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods

void PoseEstimator::findKeypoints(cv::Mat& image, std::vector<cv::KeyPoint>& keypoints)
{
    cv::SimpleBlobDetector::Params blobParams;

    // Change thresholds
//    blobParams.minThreshold = 10;
//    blobParams.maxThreshold = 200;

    // Filter by Area.
    blobParams.filterByArea = true;
    blobParams.minArea = 60;
//
//    // Filter by Circularity
//    blobParams.filterByCircularity = true;
//    blobParams.minCircularity = 0.01;
//
//    // Filter by Convexity
//    blobParams.filterByConvexity = true;
//    blobParams.minConvexity = 0.01;
//
//    // Filter by Inertia
//    blobParams.filterByInertia = true;
//    blobParams.minInertiaRatio = 0.01;

    blobParams.filterByColor = false;
//    blobParams.blobColor = 255;

//    blobParams.filterByArea = true;
 //   blobParams.minArea = 2;
//    blobParams.maxArea = 0;

//    blobParams.filterByCircularity = true;
//    blobParams.minCircularity = 0.0;

//    blobParams.filterByInertia = true;
//    blobParams.minInertiaRatio = 0.5;

//    blobParams.minDistBetweenBlobs = 50;

    // std::vector<cv::KeyPoint> foundKeypoints;

    /* 0: Binary
       1: Binary Inverted
       2: Threshold Truncated
       3: Threshold to Zero
       4: Threshold to Zero Inverted
     */

    threshold(image, image, 128, 255, 0);

    cv::SimpleBlobDetector detector(blobParams);
    detector.detect(image, keypoints);

//     vector<cv::Point> area;
//     vector<vector<cv::Point>> contour;
//     area.push_back(cv::Point2f(0, 450));
//     area.push_back(cv::Point2f(image.cols - 2, 450));
//     area.push_back(cv::Point2f(image.cols - 2, 750));
//     area.push_back(cv::Point2f(0, 750));
//     contour.push_back(area);
//
//     for (cv::KeyPoint &k : foundKeypoints)
//     {
//         if (pointPolygonTest(area, k.pt, false) > 0) {
//             keypoints.push_back(k);
//         }
//     }
//
//     cv::Mat imageWithKeypoints;
//     cv::drawKeypoints(image, keypoints, imageWithKeypoints, cv::Scalar(0, 0, 255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
//     cv::drawContours(imageWithKeypoints, contour, 0, cv::Scalar(0, 255, 0), 2);
//
//     cv::namedWindow("Display Image", CV_WINDOW_AUTOSIZE);
//     cv::imshow("Display Image", imageWithKeypoints);
//
//     cv::waitKey(0);

}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
PoseEstimator::PoseEstimator()
{
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
PoseEstimator::~PoseEstimator()
{
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void PoseEstimator::estimate(cv::Mat& image, std::vector<cv::KeyPoint>& keypoints)
{
    findKeypoints(image, keypoints);
}

