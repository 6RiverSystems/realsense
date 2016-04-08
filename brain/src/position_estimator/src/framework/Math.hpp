/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef MATH_HPP_
#define MATH_HPP_

#include <cmath>

namespace srs {

class Math
{
public:
    template<class TYPE>
    inline static TYPE inch2mm(TYPE inch)
    {
        return 25.4 * inch;
    }

    template<typename TYPE>
    inline static TYPE mm2inch(TYPE mm)
    {
        return mm / 25.4;
    }

    template<typename TYPE>
    static TYPE deg2rad(TYPE deg)
    {
        return (M_PI / 180) * deg;
    }

    template<typename TYPE>
    static TYPE rad2deg(TYPE rad)
    {
        return (180 / M_PI) * rad;
    }

    static cv::Mat zeros(const cv::Mat original)
    {
        return cv::Mat::zeros(original.rows, original.cols, original.type());
    }

    static cv::Mat cholesky(const cv::Mat A)
    {
        cv::Mat R = A.clone();
        if (Cholesky(R.ptr<double>(), R.step, R.cols, 0, 0, 0))
        {
            cv::Mat diagonal = R.diag();
            for (unsigned int e = 0; e < diagonal.rows; ++e)
            {
                double element = diagonal.at<double>(e);
                R.row(e) *= element;
                R.at<double>(e, e) = 1.0f / element;
            }
        }

        return R;
    }
};

} // namespace srs

#endif // MATH_HPP_
