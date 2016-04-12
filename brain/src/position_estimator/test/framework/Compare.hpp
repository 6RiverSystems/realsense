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
    static bool similar(cv::Mat source, cv::Mat expected, TYPE percentage)
    {
        if (source.rows != expected.rows || source.cols != expected.cols)
        {
            return false;
        }

        for (int i = 0; i < source.rows; ++i)
        {
            for (int j = 0; j < source.cols; ++j)
            {
            }
        }
        return false;
    }
};

} // namespace test
} // namespace srs

#endif // COMPARE_HPP_
