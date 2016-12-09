/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <srslib_framework/platform/timing/TimingDataRecorder.hpp>

namespace srs {

/**
 * This class Provides an RAII wrapper for the TimingDataRecorder
 */
class ScopedTimingSampleRecorder
{
public:
    /**
     * Constructor - This will start timing a sample
     * @param tdr a pointer the the recorder
     */
    ScopedTimingSampleRecorder(TimingDataRecorder* tdr)
    {
        if (tdr != nullptr)
        {
            tdr_ = tdr;
            tdr_->startSample();
        }
    };

    /**
     * Destructor - this will stop timing the sample if it has not yet done so
     */
    ~ScopedTimingSampleRecorder()
    {
        stopSample();
    };

    /**
     * Stop recording the current sample
     */
    void stopSample()
    {
        if (tdr_ != nullptr)
        {
            tdr_->stopSample();
            tdr_ = nullptr;
        }
    };

private:
    TimingDataRecorder* tdr_ = nullptr;
};

} // namespace srs
