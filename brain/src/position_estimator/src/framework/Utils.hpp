/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef UTILS_HPP_
#define UTILS_HPP_

#include <string>
#include <iostream>
#include <iomanip>
#include <cmath>
using namespace std;

#include <opencv2/opencv.hpp>

namespace srs {

struct Utils
{
    template<unsigned int PRECISION = 6, typename TYPE = double>
    static void print(cv::Mat matrix, string name = "")
    {
        if (!name.empty())
        {
            cout << name << " ";
        }
        cout << "(" << matrix.rows << "x" << matrix.cols << ")" << endl;

        TYPE threshold = pow(TYPE(10.0), -TYPE(PRECISION));
        for (unsigned int r = 0; r < matrix.rows; ++r)
        {
            for (unsigned int c = 0; c < matrix.cols; ++c)
            {
                TYPE value = matrix.at<TYPE>(r, c);
                value = abs(value) > threshold ? value : 0;
                cout << setfill(' ') << setw(PRECISION + 5) << scientific << setprecision(PRECISION - 3) << value;
            }
            cout << endl;
        }
    }
};

} // namespace srs

#endif // UTILS_HPP_
