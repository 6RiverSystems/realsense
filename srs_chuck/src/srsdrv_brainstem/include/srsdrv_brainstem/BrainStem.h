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
#include <srslib_framework/OdometryRPM.h>
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

	void OnButtonEvent( LED_ENTITIES eButtonId );

	void OnOperationalStateChanged( uint32_t upTime, const MOTION_STATUS_DATA& motionStatus,
		const FAILURE_STATUS_DATA& failureStatus );

	void OnVoltageChanged( float fVoltage );

private:

	void CreateSubscribers( );

	void CreatePublishers( );

	void SetupCallbacks( );

	void GetHardwareInformation( );

	void GetOperationalState( );

	void OnPing( );

	void OnChangeVelocity( const srslib_framework::OdometryRPM::ConstPtr& velocity );

	void OnRosCallback( const std_msgs::String::ConstPtr& msg );

private:

	static constexpr auto REFRESH_RATE_HZ = 100;

	static constexpr auto OPERATIONAL_STATE_TOPIC = "/info/operational_state";

	static constexpr auto VOLTAGE_TOPIC = "/info/voltage";

	static constexpr auto CONNECTED_TOPIC = "/internal/drivers/brainstem/connected";

	static constexpr auto VELOCITY_TOPIC = "/internal/sensors/odometry/rpm/cmd";

	static constexpr auto PING_TOPIC = "/internal/state/ping";

	static constexpr auto RACE_TOPIC = "/internal/command/race";

	// TODO: Remove/Replace with proper messages
	static constexpr auto COMMAND_TOPIC = "/cmd_ll";

	static constexpr auto EVENT_TOPIC = "/ll_event";

	ros::NodeHandle 			m_rosNodeHandle;

	ros::Subscriber				m_llcmdSubscriber;

	ros::Subscriber				m_pingSubscriber;

	ros::Subscriber				m_velocitySubscriber;

	ros::Publisher				m_llEventPublisher;

	ros::Publisher				m_operationalStatePublisher;

	ros::Publisher				m_voltagePublisher;

	ros::Publisher				m_connectedPublisher;

	std::shared_ptr<IO>			m_pSerialIO;

	std::shared_ptr<BrainStemEmulator> m_brainstemEmulator;

	BrainStemMessageProcessor	m_messageProcessor;
};

} // namespace srs

#endif  // BRAINSTEM_HPP_
