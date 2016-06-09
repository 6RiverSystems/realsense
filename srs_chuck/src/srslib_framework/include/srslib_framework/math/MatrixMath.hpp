/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef MATRIXMATH_HPP_
#define MATRIXMATH_HPP_

#include <cmath>
#include <limits>
using namespace std;

#include <opencv2/opencv.hpp>

#include <srslib_framework/math/BasicMath.hpp>

namespace srs {

struct MatrixMath
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
                else if (BasicMath::isNan<TYPE>(*element))
                {
                    *element = minReplacement;
                }
            }
        }
    }

    static cv::Mat zeros(const cv::Mat original)
    {
        return cv::Mat::zeros(original.rows, original.cols, original.type());
    }
};

} // namespace srs

#endif // MATRIXMATH_HPP_
