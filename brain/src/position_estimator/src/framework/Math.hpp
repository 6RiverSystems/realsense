/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef MATH_HPP_
#define MATH_HPP_

#include <cmath>

#include <opencv2/opencv.hpp>

#include "Utils.hpp"

namespace srs {

struct Math
{
    template<typename TYPE = double>
    static cv::Mat cholesky(const cv::Mat A)
    {
        cv::Mat R = A.clone();
        if (Cholesky(R.ptr<TYPE>(), R.step, R.cols, 0, 0, 0))
        {
            cv::Mat diagonal = R.diag();
            for (unsigned int e = 0; e < diagonal.rows; ++e)
            {
                TYPE element = diagonal.at<TYPE>(e);
                R.row(e) *= element;
                R.at<TYPE>(e, e) = TYPE(1.0) / element;
            }
        }

        return R;
    }

    template<typename TYPE = double>
    constexpr inline static TYPE deg2rad(TYPE deg)
    {
        return TYPE(M_PI / 180) * deg;
    }

    template<typename TYPE = double>
    constexpr inline static TYPE inch2m(TYPE inch)
    {
        return TYPE(0.0254) * inch;
    }

    template<typename TYPE = double>
    constexpr inline static TYPE inch2mm(TYPE inch)
    {
        return TYPE(25.4) * inch;
    }

    template<typename TYPE = double>
    constexpr inline static TYPE mm2inch(TYPE mm)
    {
        return mm / TYPE(25.4);
    }

    template<typename TYPE = double>
    constexpr inline static TYPE rad2deg(TYPE rad)
    {
        return TYPE(180 / M_PI) * rad;
    }

    static cv::Mat zeros(const cv::Mat original)
    {
        return cv::Mat::zeros(original.rows, original.cols, original.type());
    }
};

} // namespace srs

#endif // MATH_HPP_
