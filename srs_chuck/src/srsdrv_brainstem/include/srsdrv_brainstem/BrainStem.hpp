/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>

#include <srslib_framework/io/IO.hpp>

#include <BrainStemEmulator.hpp>
#include <BrainStemMessageProcessor.hpp>

#include <srsdrv_brainstem/RobotSetupConfig.h>

namespace srs
{

class BrainStem
{

public:

	BrainStem(const std::string& strSerialPort);

	virtual ~BrainStem();

	void run();

	void connectionChanged(bool bIsConnected);

private:

    void cfgCallback(srsdrv_brainstem::RobotSetupConfig &config, uint32_t level);

	static constexpr auto REFRESH_RATE_HZ = 100.0f;

	dynamic_reconfigure::Server<srsdrv_brainstem::RobotSetupConfig> configServer_;

	std::shared_ptr<IO>					io_;

	std::shared_ptr<BrainStemEmulator>	brainstemEmulator_;

	BrainStemMessageProcessor			messageProcessor_;

	ros::NodeHandle						nodeHandle_;

	ros::Timer							brainstemFaultTimer_;
};

} // namespace srs
