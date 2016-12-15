/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <ros/ros.h>

#include <srslib_framework/io/IO.hpp>

#include <BrainStemEmulator.hpp>
#include <BrainStemMessageProcessor.hpp>

namespace srs
{

class BrainStem
{

public:

	BrainStem(const std::string& strSerialPort);

	virtual ~BrainStem();

	void run();

	void connectionChanged(bool bIsConnected);

    void checkForBrainstemFaultTimer(const ros::TimerEvent& event);

private:

    void startFaultTimer();

    void stopFaultTimer();

	static constexpr auto REFRESH_RATE_HZ = 10.0f;

	std::shared_ptr<IO>					serialIO_;

	std::shared_ptr<BrainStemEmulator>	brainstemEmulator_;

	BrainStemMessageProcessor			messageProcessor_;

	ros::NodeHandle						nodeHandle_;

	ros::Timer							brainstemFaultTimer_;
};

} // namespace srs
