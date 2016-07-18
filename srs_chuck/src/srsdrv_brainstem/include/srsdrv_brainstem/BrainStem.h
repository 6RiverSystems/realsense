/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */

#ifndef BRAINSTEM_HPP_
#define BRAINSTEM_HPP_

#include <ros/ros.h>
#include <boost/timer.hpp>

#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include <srslib_framework/io/IO.hpp>
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

	void OnOdometryChanged( uint32_t dwTimeStamp, float fLinearVelocity, float fAngularVelocity );

	void OnHardwareInfo( uint32_t uniqueId[4], uint8_t chassisGeneration, uint8_t brainstemHwVersion,
		const std::string& strBrainstemSwVersion );

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

	void OnChangeVelocity( const geometry_msgs::Twist::ConstPtr& velocity );

	void OnRace( const geometry_msgs::Twist::ConstPtr& velocity );

	void OnRosCallback( const std_msgs::String::ConstPtr& msg );

private:

	static constexpr auto REFRESH_RATE_HZ = 100;

	static constexpr auto HARDWARE_INFO_TOPIC = "/info/hardware";

	static constexpr auto OPERATIONAL_STATE_TOPIC = "/info/operational_state";

	static constexpr auto VOLTAGE_TOPIC = "/info/voltage";

	static constexpr auto CONNECTED_TOPIC = "/internal/drivers/brainstem/connected";

	static constexpr auto VELOCITY_TOPIC = "/internal/drivers/brainstem/cmd_velocity";

	static constexpr auto ODOMETRY_TOPIC = "/internal/sensors/odometry/raw";

	static constexpr auto PING_TOPIC = "/internal/state/ping";
	
	static constexpr auto RACE_TOPIC = "/internal/command/race";

	// TODO: Remove/Replace with proper messages
	static constexpr auto COMMAND_TOPIC = "/cmd_ll";

	static constexpr auto EVENT_TOPIC = "/ll_event";

	ros::NodeHandle 			m_rosNodeHandle;

	ros::Subscriber				m_llcmdSubscriber;

	ros::Subscriber				m_raceSubscriber;

	ros::Subscriber				m_pingSubscriber;

	ros::Subscriber				m_velocitySubscriber;

	ros::Publisher				m_llEventPublisher;

	ros::Publisher				m_hardwareInfoPublisher;

	ros::Publisher				m_operationalStatePublisher;

	ros::Publisher				m_voltagePublisher;

	ros::Publisher				m_connectedPublisher;

	ros::Publisher				m_odometryRawPublisher;

	boost::timer				m_raceTimer;

	geometry_msgs::Twist::Ptr	m_raceVelocity;

	std::shared_ptr<IO>			m_pSerialIO;

	BrainStemMessageProcessor	m_messageProcessor;

	uint32_t					m_dwLastOdomTime;

	ros::Time					m_rosOdomTime;

};

} // namespace srs

#endif  // BRAINSTEM_HPP_
