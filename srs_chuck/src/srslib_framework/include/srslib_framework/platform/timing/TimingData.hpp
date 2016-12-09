/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <string>
#include <ros/ros.h>

namespace srs {

/**
 * Helper struct to hold a single datum of duration information.
 */
struct DurationDatum
{
    /**
     * Constructor
     * @param endTime the end time of the duration sample
     * @param duration the duration of the sample
     */
    DurationDatum(double endTime, double duration) :
        endTime_(endTime), duration_(duration)
        {};

    /**
     * Destructor
     */
    ~DurationDatum()
    {};

    double endTime_;
    double duration_;
};

/**
 * Holds timing data
 */
class TimingData
{
public:
    /**
     * Constructor
     * @param id the id of the recorder of this data
     */
    TimingData(std::string id) :
        id_(id)
    {}

    /**
     * Add a sample to the list of recorded samples
     * @param endTime the end time of the sample
     * @param duration the duration of the sample
     */
    void addSample(double endTime, double duration)
    {
        samples_.push_back(DurationDatum(endTime, duration));
    }

    /**
     * Check if this set of samples has started recording any data
     * @return true if the set has started recording
     */
    bool hasStarted()
    {
        return startTime_ >= 0.0;
    }

    /**
     * Set the start time of the set of samples
     * @param startTime the start time
     */
    void setStartTime(double startTime)
    {
        startTime_ = startTime;
    }

    /**
     * Get the start time of this set of samples
     * @return the start time
     */
    double getStartTime() const
    {
        return startTime_;
    }

    /**
     * Set the end time of the set of samples
     * @param endTime the end time
     */
    void setEndTime(double endTime)
    {
        endTime_ = endTime;
    }

    /**
     * Get the end time of this set of samples
     * @return the end time
     */
    double getEndTime() const
    {
        return endTime_;
    }

    /**
     * Get the set of samples
     * @return the samples
     */
    std::vector<DurationDatum> getSamples() const
    {
        return samples_;
    }

    /**
     * Get the id
     * @return the id
     */
    std::string getId() const
    {
        return id_;
    }

    /**
     * Reset this timing data record
     */
    void reset()
    {
        startTime_ = -1;
        endTime_ = -1;
        samples_.clear();
    }

    /**
     * Get data for use in a log
     */
    std::string getDataForLog() const
    {
        double meanVal = 0;
        double maxVal = 0;
        if (samples_.size() > 0)
        {
            for (auto s : samples_)
            {
                meanVal += s.duration_;
                if (s.duration_ > maxVal)
                {
                    maxVal = s.duration_;
                }
            }
            meanVal /= samples_.size();
        }
        std::stringstream ss;
        ss << "Timing for: " << id_ << ".  Mean: " << meanVal << ", max: " << maxVal;
        return ss.str();
    }

private:
    std::string id_;
    std::vector<DurationDatum> samples_;
    double startTime_ = -1;
    double endTime_ = -1;
};

} // namespace srs
