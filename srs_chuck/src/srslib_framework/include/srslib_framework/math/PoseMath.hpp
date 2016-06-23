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
    template<typename TYPE = double>
    inline static Pose<TYPE> add(Pose<TYPE> p1, Pose<TYPE> p2)
    {
        return Pose<TYPE>(
            p1.x + p2.x,
            p1.y + p2.y,
            AngleMath::normalizeAngleRad(p1.theta + p2.theta));
    }

    template<typename TYPE = double>
    inline static TYPE euclidean(Pose<TYPE> p1, Pose<TYPE> p2)
    {
        TYPE deltaX = p1.x - p2.x;
        TYPE deltaY = p1.y - p2.y;
        return sqrt(deltaX * deltaX + deltaY * deltaY);
    }

    template<typename TYPE = double>
    inline static TYPE euclidean2(Pose<TYPE> p1, Pose<TYPE> p2)
    {
        TYPE deltaX = p1.x - p2.x;
        TYPE deltaY = p1.y - p2.y;
        return deltaX * deltaX + deltaY * deltaY;
    }

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

    template<typename TYPE = double>
    inline static Pose<TYPE> transform(Pose<TYPE> p, TYPE distance)
    {
        return Pose<TYPE>(
            p.x + distance * cos(p.theta),
            p.y + distance * sin(p.theta),
            p.theta);
    }
};

} // namespace srs

#endif // POSEMATH_HPP_
