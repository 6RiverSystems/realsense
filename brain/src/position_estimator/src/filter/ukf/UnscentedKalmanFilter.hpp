/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef UNSCENTEDKALMANFILTER_HPP_
#define UNSCENTEDKALMANFILTER_HPP_

#include <opencv2/opencv.hpp>

namespace sixrs {

template<int TYPE>
class UnscentedKalmanFilter
{
public:
    UnscentedKalmanFilter(unsigned int n, double alpha, double beta);
    ~UnscentedKalmanFilter();

    void reset();

private:
    unsigned int n_;
    double alpha_;
    double beta_;
    double kappa_;
    double lambda_;
    double c_;

    cv::Mat WM_;
    cv::Mat WC_;

    cv::Mat state_;

    void initializeWeights();
};

} // namespace sixrs

// This include is to insert the actual template code in the file without
// specifying all the code in the header file.
#include "UnscentedKalmanFilter.cpp"

#endif // UNSCENTEDKALMANFILTER_HPP_
