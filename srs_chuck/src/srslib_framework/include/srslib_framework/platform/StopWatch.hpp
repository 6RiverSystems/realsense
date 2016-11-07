/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <chrono>
using namespace std::chrono;

namespace srs {

class StopWatch
{
    typedef duration<double, ratio<1>> DurationType;

public:
    StopWatch() :
        time_(high_resolution_clock::now())
    {}

    double elapsed() const
    {
        return duration_cast<DurationType>(high_resolution_clock::now() - time_).count();
    }

    void reset()
    {
        time_ = high_resolution_clock::now();
    }

private:
    time_point<high_resolution_clock> time_;
};

} // namespace srs
