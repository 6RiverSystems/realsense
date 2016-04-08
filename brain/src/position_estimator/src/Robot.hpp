/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef ROBOT_HPP_
#define ROBOT_HPP_

#include <opencv2/opencv.hpp>

#include <filter/ProcessModel.hpp>
#include <filter/FilterState.hpp>

namespace srs {

class Robot : public ProcessModel<>
{
public:
    Robot() :
        ProcessModel<>()
    {}

    virtual ~Robot()
    {}

    virtual cv::Mat transform(const cv::Mat state, Command<>* const command, double DT);

private:
};

} // namespace srs

#endif // ROBOT_HPP_
