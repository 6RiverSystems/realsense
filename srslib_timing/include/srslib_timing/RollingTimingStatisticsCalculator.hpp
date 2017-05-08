/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <srslib_timing/RosStopWatch.hpp>

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/median.hpp>
#include <boost/accumulators/statistics/max.hpp>
#include <boost/accumulators/statistics/min.hpp>

namespace srs {


class RollingTimingStatisticsCalculator
{
public:
    RollingTimingStatisticsCalculator(unsigned int sampleSetSize_ = 10) :
        sampleSetSize_(sampleSetSize_)
    {};

    ~RollingTimingStatisticsCalculator()
    {};

    void startSample()
    {
        stopWatch_.reset();
        activeSample_ = true;
    };

    void stopSample()
    {
        if (!activeSample_)
        {
            return;
        }

        double duration = stopWatch_.elapsedSeconds();
        activeSample_ = false;

        // Record the duration and update statistics
        addSample(duration);
    }
    bool isValid()
    {
        return storedDurations_.size() > 0;
    }
    double getMedian()
    {
        return boost::accumulators::median(accumulator_);
    }

    double getMin()
    {
        return boost::accumulators::min(accumulator_);
    }

    double getMean()
    {
        return boost::accumulators::mean(accumulator_);
    }

    double getMax()
    {
        return boost::accumulators::max(accumulator_);
    }

private:
    void addSample(double sample)
    {
        while (storedDurations_.size() >= sampleSetSize_)
        {
            storedDurations_.pop_front();
        }
        storedDurations_.push_back(sample);

        // Reset the accumulator and push in the data
        accumulator_ = accumulator_t();
        for (auto iter = storedDurations_.begin(); iter != storedDurations_.end(); ++iter)
        {
            accumulator_(*iter);
        }
    }

    typedef boost::accumulators::accumulator_set<float,
        boost::accumulators::features<
            boost::accumulators::tag::min,
            boost::accumulators::tag::max,
            boost::accumulators::tag::mean,
            boost::accumulators::tag::median>
    > accumulator_t;

    accumulator_t accumulator_;

    RosStopWatch stopWatch_;
    unsigned int sampleSetSize_ = 10;
    bool activeSample_ = false;

    std::list<double> storedDurations_;
};


} // namespace srs
