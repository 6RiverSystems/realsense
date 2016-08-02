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
#include <srslib_framework/math/Base2Ocv.hpp>

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
    inline static TYPE dot(Pose<TYPE> p1, Pose<TYPE> p2)
    {
        return p1.x * p2.x + p1.y * p2.y;
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

    template<typename TYPE = double>
    inline static bool intersection(vector<Pose<TYPE>>& rectangle, Pose<TYPE> p)
    {
        cv::Mat P0 = poseXY2mat(rectangle[0]);
        cv::Mat P1 = poseXY2mat(rectangle[1]);
        cv::Mat P2 = poseXY2mat(rectangle[2]);
        cv::Mat P3 = poseXY2mat(rectangle[3]);

        cv::Mat P = poseXY2mat(p);

        cv::Mat P0_P3 = P0 - P3;
        cv::Mat P2_P3 = P2 - P3;

        cv::Mat PC = 2.0 * P - P0 - P2;

        return (P2_P3.dot(PC - P2_P3) <= 0.0 && P2_P3.dot(PC + P2_P3) >= 0.0) &&
               (P0_P3.dot(PC - P0_P3) <= 0.0 && P0_P3.dot(PC + P0_P3) >= 0.0);
    }

    template<typename TYPE = double>
    inline static TYPE measureAngle(Pose<TYPE> p1, Pose<TYPE> p2)
    {
        return AngleMath::normalizeAngleRad(
            acos(PoseMath::dot<TYPE>(p1, p2) /
                (PoseMath::norm<TYPE>(p1) * PoseMath::norm<TYPE>(p2))));
    }

    template<typename TYPE = double>
    inline static TYPE norm(Pose<TYPE> p)
    {
        return sqrt(PoseMath::dot<TYPE>(p, p));
    }

    template<typename TYPE = double>
    static cv::Mat pose2mat(const Pose<TYPE> pose)
    {
        cv::Mat R = cv::Mat::zeros(3, 1, Base2Ocv<TYPE>::OCV_TYPE);
        R.at<TYPE>(0) = pose.x;
        R.at<TYPE>(1) = pose.y;
        R.at<TYPE>(2) = pose.theta;

        return R;
    }

    template<typename TYPE = double>
    static cv::Mat poseXY2mat(const Pose<TYPE> pose)
    {
        cv::Mat R = cv::Mat::zeros(2, 1, Base2Ocv<TYPE>::OCV_TYPE);
        R.at<TYPE>(0) = pose.x;
        R.at<TYPE>(1) = pose.y;

        return R;
    }

    template<typename TYPE = double>
    static vector<Pose<>> pose2polygon(const Pose<TYPE> center, TYPE offset, TYPE width, TYPE depth)
    {
        Pose<> reflection = PoseMath::rotate(center, M_PI);
        Pose<> p = PoseMath::transform(reflection, offset);

        reflection = PoseMath::rotate(p, M_PI_2);
        Pose<> p0 = PoseMath::transform(reflection, width / 2);

        reflection = PoseMath::rotate(p, -M_PI_2);
        Pose<> p1 = PoseMath::transform(reflection, width / 2);

        reflection = p1;
        reflection.theta = center.theta;
        Pose<> p2 = PoseMath::transform(reflection, depth);

        reflection = p0;
        reflection.theta = center.theta;
        Pose<> p3 = PoseMath::transform(reflection, depth);

        vector<Pose<>> polygon;
        polygon.clear();
        polygon.push_back(p0);
        polygon.push_back(p1);
        polygon.push_back(p2);
        polygon.push_back(p3);

        return polygon;
    }

    /**
     * @brief Perform a pose rotation around itself.
     *
     * @tparam TYPE Basic type of the operation
     *
     * @param p pose to rotate
     * @param angle Angle of the rotation
     *
     * @return rotated pose with theta normalized angle to a [-pi, pi) range
     */
    template<typename TYPE = double>
    inline static Pose<TYPE> rotate(Pose<TYPE> p, TYPE angle)
    {
        return Pose<TYPE>(
            p.x,
            p.y,
            AngleMath::normalizeAngleRad(p.theta + angle));
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
