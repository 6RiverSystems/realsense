/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <string>
#include <unordered_map>
#include <srslib_timing/TimingDataRecorder.hpp>

namespace srs {


/**
 * Class that can manage a group of timing data recorders.
 */
class MasterTimingDataRecorder
{
public:
    /**
     * Constructor
     * @param idPrefix a prefix to use for the data id for all recorders
     */
    MasterTimingDataRecorder(std::string idPrefix = "") :
        idPrefix_(idPrefix)
    {};

    /**
     * Destructor
     */
    ~MasterTimingDataRecorder()
    {
        for (auto r : recorderMap_)
        {
            delete r.second;
        }
    };

    /**
     * Get a pointer to a recorder with the given id.
     *   If one does not exist, it creates a new one.
     * @param id the id to use for lookup
     * @param sampleSetLength the length of time to collect samples before publishing
     * @return a pointer to a recorder
     */
    TimingDataRecorder* getRecorder(std::string id, double sampleSetLength = 1.0)
    {
        std::string fullId = idPrefix_ + id;

        RecorderMap::const_iterator iter = recorderMap_.find(fullId);
        if (iter != recorderMap_.end())
        {
            return iter->second;
        }

        // No match, create a new one.
        TimingDataRecorder* recorder = new TimingDataRecorder(fullId, sampleSetLength);
        recorderMap_[fullId] = recorder;

        return recorder;
    };

private:
    typedef std::unordered_map<std::string, TimingDataRecorder*> RecorderMap;

    RecorderMap recorderMap_;
    std::string idPrefix_;
};


} // namespace srs
