/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */

#ifndef BRAINSTEM_HPP_
#define BRAINSTEM_HPP_

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <srslib_framework/io/IO.hpp>
#include <srslib_framework/ros/channel/ChannelBrainstemConnected.hpp>
#include <BrainStemEmulator.h>
#include <BrainStemMessageProcessor.h>

namespace srs
{

class BrainStem
{

public:

	BrainStem( const std::string& strSerialPort );

	virtual ~BrainStem( );

	void Run( );

	void OnConnectionChanged( bool bIsConnected );

private:

	static constexpr auto REFRESH_RATE_HZ = 100;

	std::shared_ptr<IO>					m_pSerialIO;

	ChannelBrainstemConnected			connectedChannel_;

	std::shared_ptr<BrainStemEmulator>	m_brainstemEmulator;

	BrainStemMessageProcessor			m_messageProcessor;
};

} // namespace srs

#endif  // BRAINSTEM_HPP_
