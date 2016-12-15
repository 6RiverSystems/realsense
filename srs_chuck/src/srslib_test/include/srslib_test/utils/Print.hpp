/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <string>
#include <iostream>
#include <iomanip>
#include <cmath>
using namespace std;

#include <opencv2/opencv.hpp>

namespace srs {
namespace test {

struct Print
{
    template<unsigned int PRECISION = 6, typename TYPE = double>
    static void print(cv::Mat matrix, string name = "")
    {
        cout << printToString<PRECISION, TYPE>(matrix, name) << endl;
    }

    template<unsigned int PRECISION = 6, typename TYPE = double>
    static string printToString(cv::Mat matrix, string name = "")
    {
        ostringstream output;

        if (!name.empty())
        {
            output << name << " ";
        }
        output << "(" << matrix.rows << "x" << matrix.cols << ") {" << endl;

        TYPE threshold = pow(TYPE(10.0), -TYPE(PRECISION));
        for (unsigned int r = 0; r < matrix.rows; ++r)
        {
            for (unsigned int c = 0; c < matrix.cols; ++c)
            {
                TYPE value = matrix.at<TYPE>(r, c);
                if (abs(value) > threshold)
                {
                    output << setfill(' ') << setw(PRECISION + 5) << scientific <<
                        setprecision(PRECISION - 3) << value;
                }
                else
                {
                    output << setfill(' ') << setw(PRECISION + 5) << "0";
                }
            }
            output << endl;
        }
        output << "}";

        return output.str();
    }

    template<unsigned int PRECISION = 9, typename TYPE = float>
    static string printToString(TYPE* matrix,
        unsigned int columns, unsigned int rows,
        TYPE threshold = 1e10)
    {
        ostringstream output;

        output << setw(PRECISION + 1) << "  ";
        for (int col = 0; col < columns; ++col)
        {
            output << setw(PRECISION + 1) << col;
        }

        output << endl;
        for (int row = rows - 1; row >= 0; row--)
        {
            output << setw(PRECISION + 1) << row;
            for (int col = 0; col < columns; ++col)
            {
                TYPE value = *(matrix + (row * columns) + col);

                if (value < threshold)
                {
                    output << setfill(' ') << setw(PRECISION + 1) << scientific <<
                        setprecision(PRECISION - 7) << value;
                }
                else
                {
                    output << setw(PRECISION + 1) << "~";
                }
            }

            output << endl;
        }

        return output.str();
    }
};

} // namespace test
} // namespace srs
