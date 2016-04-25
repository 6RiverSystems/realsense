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

#include <tap/odometry/Odometry.hpp>
#include <tap/vel_cmd/VelCmd.hpp>

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
        cout << "(" << matrix.rows << "x" << matrix.cols << ") {" << endl;

        TYPE threshold = pow(TYPE(10.0), -TYPE(PRECISION));
        for (unsigned int r = 0; r < matrix.rows; ++r)
        {
            for (unsigned int c = 0; c < matrix.cols; ++c)
            {
                TYPE value = matrix.at<TYPE>(r, c);
                if (abs(value) > threshold)
                {
                    cout << setfill(' ') << setw(PRECISION + 5) << scientific <<
                        setprecision(PRECISION - 3) << value;
                }
                else
                {
                    cout << setfill(' ') << setw(PRECISION + 5) << "0";
                }
            }
            cout << endl;
        }
        cout << "}" << endl;
    }

    template<typename TYPE>
    static void print(TYPE object)
    {
        cout << object.toString() << endl;
    }
};

} // namespace srs

#endif // UTILS_HPP_
