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

	void OnHardwareInfo( uint16_t uniqueId, uint8_t bodyType, uint32_t configuration,
		uint32_t lifetimeHours, uint32_t lifetimeMeters, uint32_t batteryHours,
		uint32_t wheelMeters, const std::string& strBrainstemVersion );

	void OnOperationalStateChanged( uint32_t upTime, MOTION_STATUS_DATA motionStatus,
		FAILURE_STATUS_DATA failureStatus, uint8_t suspendState );

	void OnVoltageChanged( float fVoltage );

private:

	void CreateSubscribers( );

	void CreatePublishers( );

	void SetupCallbacks( );

	void GetHardwareInformation( );

	void GetOperationalState( );

	void OnPing( );

	void OnChangeVelocity( const geometry_msgs::Twist::ConstPtr& velocity );

	void OnRosCallback( const std_msgs::String::ConstPtr& msg );

private:

	ros::NodeHandle 			m_rosNodeHandle;

	ros::Subscriber				m_llcmdSubscriber;

	ros::Subscriber				m_pingSubscriber;

	ros::Subscriber				m_velocitySubscriber;

	ros::Publisher				m_llEventPublisher;

	ros::Publisher				m_OdometryRawPublisher;

	ros::Publisher				m_ConnectedPublisher;

	std::shared_ptr<IO>			m_pSerialIO;

	BrainStemMessageProcessor	m_messageProcessor;

	uint32_t					m_dwLastOdomTime;

	ros::Time					m_rosOdomTime;

};

} // namespace srs

#endif  // BRAINSTEM_HPP_
