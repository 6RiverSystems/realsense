/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef MATH_HPP_
#define MATH_HPP_

#include <cmath>
#include <limits>
using namespace std;

#include <opencv2/opencv.hpp>

namespace srs {

struct Math
{
    template<typename TYPE = double>
    static cv::Mat cholesky(const cv::Mat A)
    {
        cv::Mat R = A.clone();
        int n = R.cols;
        for (int i = 0; i < n; ++i)
        {
            for (int j = i; j < n; ++j)
            {
                TYPE sum = R.at<TYPE>(i, j);
                for (int k = i - 1; k >= 0; --k)
                {
                    sum -= R.at<TYPE>(i, k) * R.at<TYPE>(j, k);
                }
                if (i == j)
                {
                    R.at<TYPE>(i, i) = sqrt(sum);
                }
                else
                {
                    R.at<TYPE>(j, i) = sum / R.at<TYPE>(i, i);
                }
            }
        }
        for (int i = 0; i < n; ++i)
        {
            for (int j = 0; j < i; ++j)
            {
                R.at<TYPE>(j, i) = TYPE();
            }
        }

        return R;
    }

    template<typename TYPE = double>
    static void checkDiagonal(cv::Mat& A,
        const TYPE minValue = numeric_limits<TYPE>::min(),
        const TYPE minReplacement = numeric_limits<TYPE>::min(),
        const TYPE maxValue = numeric_limits<TYPE>::max(),
        const TYPE maxReplacement = numeric_limits<TYPE>::max())
    {
        for (unsigned int e = 0; e < A.rows; ++e)
        {
            TYPE* element = A.ptr<TYPE>(e, e);
            if (abs(*element) < minValue)
            {
                *element = minReplacement;
            }
            else if (abs(*element) > maxValue)
            {
                *element = maxReplacement;
            }
        }
    }

    template<typename TYPE = double>
    static void checkRange(cv::Mat& A,
        const TYPE minValue = numeric_limits<TYPE>::min(),
        const TYPE minReplacement = numeric_limits<TYPE>::min(),
        const TYPE maxValue = numeric_limits<TYPE>::max(),
        const TYPE maxReplacement = numeric_limits<TYPE>::max())
    {
        for (int i = 0; i < A.rows; ++i)
        {
            for (int j = 0; j < A.cols; ++j)
            {
                TYPE* element = A.ptr<TYPE>(i, j);
                if (abs(*element) < minValue)
                {
                    *element = minReplacement;
                }
                else if (abs(*element) > maxValue)
                {
                    *element = maxReplacement;
                }
                else if (isNan(*element))
                {
                    *element = minReplacement;
                }
            }
        }
    }

    template<typename TYPE = double>
    constexpr inline static TYPE deg2rad(TYPE deg)
    {
        return deg * TYPE(M_PI) / TYPE(180);
    }

    template<typename TYPE = double>
    inline static TYPE euclidean(TYPE x1, TYPE y1, TYPE x2, TYPE y2)
    {
        TYPE deltaX = x2 - x1;
        TYPE deltaY = y2 - y1;
        return sqrt(deltaX * deltaX + deltaY * deltaY);
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

    constexpr inline static bool isNan(double value)
    {
        return value != value;
    }

    constexpr inline static bool isNan(float value)
    {
        return value != value;
    }

    template<typename TYPE = double>
    constexpr inline static TYPE mm2inch(TYPE mm)
    {
        return mm / TYPE(25.4);
    }

    inline static unsigned int noOverflowAdd(unsigned int a, unsigned int b)
    {
        if (a > numeric_limits<unsigned int>::max() - b)
        {
            return numeric_limits<unsigned int>::max();
        }
        return a + b;
    }

    template<typename TYPE = int>
    inline static TYPE normalizeAngleDeg(TYPE deg)
    {
        return deg < 0 ? 360 - (abs(deg) % 360) : deg % 360;
    }

    inline static unsigned int normalizeRad2deg90(double rad)
    {
        double angle = Math::rad2deg<double>(rad);
        double ratio = angle / 90.0;
        double upper = ceil(ratio) * 90.0;
        double lower = floor(ratio) * 90.0;

        angle = (upper - angle) > (angle - lower) ? lower : upper;
        return Math::normalizeAngleDeg(static_cast<int>(angle));
    }

    template<typename TYPE = double>
    constexpr inline static TYPE normalizeAngleRad(TYPE rad)
    {
        return rad - 2 * M_PI * floor((rad + M_PI) / (2 * M_PI));
    }

    template<typename TYPE = double>
    constexpr inline static TYPE rad2deg(TYPE rad)
    {
        return rad * TYPE(180) / TYPE(M_PI);
    }

    template <typename TYPE = double>
    constexpr inline static TYPE sgn(TYPE value)
    {
        return TYPE((TYPE(0) < value) - (value < TYPE(0)));
    }

    static cv::Mat zeros(const cv::Mat original)
    {
        return cv::Mat::zeros(original.rows, original.cols, original.type());
    }
};

} // namespace srs

#endif // MATH_HPP_
