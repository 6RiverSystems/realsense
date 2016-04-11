/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef ROBOT_HPP_
#define ROBOT_HPP_

#include <opencv2/opencv.hpp>

#include <framework/Math.hpp>

#include <filter/Process.hpp>

namespace srs {

template<unsigned int STATE_SIZE = 5, int TYPE = CV_64F>
class Robot : public Process<STATE_SIZE, TYPE>
{
public:
    typedef typename Process<STATE_SIZE, TYPE>::BaseType BaseType;

    Robot() :
        Process<STATE_SIZE, TYPE>(cv::Mat::diag(Q))
    {}

    virtual ~Robot()
    {}

    virtual cv::Mat transformWithAB(const cv::Mat state,
        Command<TYPE>* const command,
        BaseType dT);

private:
    const static cv::Mat Q;
};

} // namespace srs

#include "Robot.cpp"

#endif // ROBOT_HPP_
