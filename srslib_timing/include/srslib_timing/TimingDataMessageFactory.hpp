/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <string>
#include <vector>
#include <utility>

#include <ros/ros.h>
#include <srslib_timing/TimingDataSample.h>
#include <srslib_timing/TimingData.hpp>

namespace srs {

struct TimingDataMessageFactory
{

    static srslib_timing::TimingData data2Msg(const TimingData& data)
    {
        srslib_timing::TimingData td;
        td.id = data.getId();
        td.start_time = ros::Time(data.getStartTime());
        td.end_time = ros::Time(data.getEndTime());
        auto samples = data.getSamples();
        td.samples.reserve(samples.size());
        for (auto s : samples)
        {
            srslib_timing::TimingDataSample tds;
            tds.duration = s.duration_;
            tds.sample_end_time = ros::Time(s.endTime_);
            td.samples.push_back(tds);
        }
        return td;
    }
};

} // namespace srs
