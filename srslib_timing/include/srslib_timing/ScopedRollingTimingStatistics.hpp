/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <srslib_timing/RollingTimingStatisticsCalculator.hpp>

namespace srs {

/**
 * This class Provides an RAII wrapper for the RollingTimingStatisticsCalculator
 */
class ScopedRollingTimingStatistics
{
public:
    /**
     * Constructor - This will start timing a sample
     * @param tdr a pointer the the recorder
     */
    ScopedRollingTimingStatistics(RollingTimingStatisticsCalculator* rtsc)
    {
        if (rtsc != nullptr)
        {
            rtsc_ = rtsc;
            rtsc_->startSample();
        }
    };

    /**
     * Destructor - this will stop timing the sample if it has not yet done so
     */
    ~ScopedRollingTimingStatistics()
    {
        stopSample();
    };

    /**
     * Stop recording the current sample
     */
    void stopSample()
    {
        if (rtsc_ != nullptr)
        {
            rtsc_->stopSample();
            rtsc_ = nullptr;
        }
    };

private:
    RollingTimingStatisticsCalculator* rtsc_ = nullptr;
};

} // namespace srs
