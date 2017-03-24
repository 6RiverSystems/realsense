/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <chrono>
using namespace std;

namespace srs {

class StopWatch
{
public:
    StopWatch() :
        time_(chrono::high_resolution_clock::now())
    {}

    double elapsedMicroseconds() const
    {
        return chrono::duration_cast<chrono::microseconds>(
            chrono::high_resolution_clock::now() - time_).count();
    }

    double elapsedMilliseconds() const
    {
        return chrono::duration_cast<chrono::milliseconds>(
            chrono::high_resolution_clock::now() - time_).count();
    }

    void reset()
    {
        time_ = chrono::high_resolution_clock::now();
    }

private:
    chrono::time_point<chrono::high_resolution_clock> time_;
};

} // namespace srs


