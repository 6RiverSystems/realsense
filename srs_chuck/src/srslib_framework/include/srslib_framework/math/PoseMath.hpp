/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef POSEMATH_HPP_
#define POSEMATH_HPP_

#include <cmath>
#include <limits>
using namespace std;

#include <opencv2/opencv.hpp>

#include <srslib_framework/robotics/Pose.hpp>

namespace srs {

struct PoseMath
{
    // TODO: Better implement this function. The type of R should not be fixed but
    // depending on TYPE (ocv2base in reverse)
    template<typename TYPE = double>
    static cv::Mat pose2mat(const Pose<TYPE> pose)
    {
        cv::Mat R = cv::Mat::zeros(3, 1, CV_64F);
        R.at<TYPE>(0) = pose.x;
        R.at<TYPE>(1) = pose.y;
        R.at<TYPE>(2) = pose.theta;

        return R;
    }
};

} // namespace srs

#endif // POSEMATH_HPP_