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
#include <srslib_framework/TimingData.h>
#include <srslib_framework/TimingDataSample.h>

namespace srs {

struct TimingDataMessageFactory
{
    /**
     * @brief Convert a Imu message type into a Imu.
     *
     * @param message Imu to convert
     *
     * @return Imu generated from the specified Imu message
     */
    static srslib_framework::TimingData buildMessage( double startTime,
        double stopTime, const std::vector<std::pair<double, double>>& samples)
    {
        srslib_framework::TimingData td;
        td.start_time = ros::Time(startTime);
        td.start_time = ros::Time(stopTime);
        td.samples.reserve(samples.size());
        for (auto s : samples)
        {
            srslib_framework::TimingDataSample tds;
            tds.duration = s.first;
            tds.sample_end_time = ros::Time(s.second);
            td.samples.push_back(tds);
        }
        return td;
    }
};

} // namespace srs
