#ifndef _POSEESTIMATOR_HPP_
#define _POSEESTIMATOR_HPP_

#include <vector>
#include <opencv2/opencv.hpp>

class PoseEstimator
{
public:
    PoseEstimator();
    ~PoseEstimator();

    void estimate(cv::Mat& image, std::vector<cv::KeyPoint>& keypoints);

private:
    void findKeypoints(cv::Mat& image, std::vector<cv::KeyPoint>& keypoints);
};

#endif // _POSEESTIMATOR_HPP_
