/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef COMPARE_HPP_
#define COMPARE_HPP_

#include <opencv2/opencv.hpp>

namespace srs {
namespace test {

struct Compare
{
    template<typename TYPE = double>
    static bool similar(cv::Mat source, cv::Mat expected, TYPE threshold)
    {
        if (source.rows != expected.rows)
        {
            ADD_FAILURE() << "The matrices have a different row count.";
            return false;
        }

        if (source.cols != expected.cols)
        {
            ADD_FAILURE() << "The matrices have a different column count.";
            return false;
        }

        for (int i = 0; i < source.rows; ++i)
        {
            for (int j = 0; j < source.cols; ++j)
            {
                if (abs(source.at<TYPE>(i, j) - expected.at<TYPE>(i, j)) > threshold)
                {
                    ADD_FAILURE() << "Element (" << i << ", " << j << ") = " <<
                        source.at<TYPE>(i, j) <<
                        " is not as expected (" << expected.at<TYPE>(i, j) << ") " <<
                        " Threshold = " << threshold;
                    return false;
                }
            }
        }

        return true;
    }
};

} // namespace test
} // namespace srs

#endif // COMPARE_HPP_
