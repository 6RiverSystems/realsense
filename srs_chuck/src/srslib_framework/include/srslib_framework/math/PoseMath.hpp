/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

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
            AngleMath::normalizeRad(p1.theta + p2.theta));
    }

    template<typename TYPE = double>
    inline static TYPE dot(Pose<TYPE> p1, Pose<TYPE> p2)
    {
        return p1.x * p2.x + p1.y * p2.y;
    }

    /**
     * @brief Return TRUE if the poses are similar. In order to consider them
     * similar:
     *
     * - The difference between x must be less than 0.001 m
     * - The difference between angles must be less than 0.005 deg
     *
     * @param lhv Left hand pose
     * @param rhv Right-hand pose
     *
     * @return TRUE if the two poses are similar
     */
    template<typename TYPE = double>
    inline static bool equal(const Pose<TYPE>& lhv, const Pose<TYPE>& rhv)
    {
        return BasicMath::equal(lhv.x, rhv.x, 0.001) &&
            BasicMath::equal(lhv.y, rhv.y, 0.001) &&
            AngleMath::equalRad(lhv.theta, rhv.theta, 0.001);
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

        cv::Mat PC = TYPE(2) * P - P0 - P2;

        return (P2_P3.dot(PC - P2_P3) <= 0.0 && P2_P3.dot(PC + P2_P3) >= 0.0) &&
               (P0_P3.dot(PC - P0_P3) <= 0.0 && P0_P3.dot(PC + P0_P3) >= 0.0);
    }

    template<typename TYPE = double>
    inline static TYPE measureAngle(Pose<TYPE> p1, Pose<TYPE> p2)
    {
        return AngleMath::normalizeRad(
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
    static vector<Pose<>> pose2Polygon(const Pose<TYPE> center,
        TYPE forward, TYPE sideway,
        TYPE width, TYPE depth)
    {
        // Translate by the offset
        Pose<> shiftedCenter = PoseMath::translate(center, forward, sideway);

        Pose<> p0 = PoseMath::translate(shiftedCenter, depth / TYPE(2), -width / TYPE(2));
        Pose<> p1 = PoseMath::translate(shiftedCenter, depth / TYPE(2), width / TYPE(2));
        Pose<> p2 = PoseMath::translate(shiftedCenter, -depth / TYPE(2), width / TYPE(2));
        Pose<> p3 = PoseMath::translate(shiftedCenter, -depth / TYPE(2), -width / TYPE(2));

        vector<Pose<>> polygon;
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
            AngleMath::normalizeRad(p.theta + angle));
    }

    template<typename TYPE = double>
    inline static Pose<TYPE> translate(Pose<TYPE> p, TYPE forward, TYPE sideway)
    {
        return Pose<TYPE>(
            p.x + forward * cos(p.theta) - sideway * sin(p.theta),
            p.y + forward * sin(p.theta) + sideway * cos(p.theta),
            p.theta);
    }
};

} // namespace srs
