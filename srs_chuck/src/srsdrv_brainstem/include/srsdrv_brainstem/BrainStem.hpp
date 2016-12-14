/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */

#ifndef BRAINSTEM_HPP_
#define BRAINSTEM_HPP_

#include <ros/ros.h>

#include <srslib_framework/io/IO.hpp>

#include <BrainStemEmulator.hpp>
#include <BrainStemMessageProcessor.hpp>

namespace srs
{

class BrainStem
{

public:

	BrainStem( const std::string& strSerialPort );

	virtual ~BrainStem( );

	void Run( );

	void OnConnectionChanged( bool bIsConnected );

    void checkForBrainstemFaultTimer(const ros::TimerEvent& event);

private:

    void startFaultTimer();

    void stopFaultTimer();

	static constexpr auto REFRESH_RATE_HZ = 10.0f;

	std::shared_ptr<IO>					m_pSerialIO;

	std::shared_ptr<BrainStemEmulator>	m_brainstemEmulator;

	BrainStemMessageProcessor			m_messageProcessor;

	ros::NodeHandle						nodeHandle_;

	ros::Timer							brainstemFaultTimer_;
};

} // namespace srs

#endif  // BRAINSTEM_HPP_
