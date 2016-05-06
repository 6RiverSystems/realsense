/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */

#ifndef BRAINSTEM_HPP_
#define BRAINSTEM_HPP_

#include <ros/ros.h>
#include <std_msgs/String.h>
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

	void OnArrived( );

	void OnButtonEvent( ENTITIES eButtonId );

	void OnOdometryChanged( uint32_t dwTimeStamp, float fLinearVelocity, float fAngularVelocity );

private:

	void OnChangeVelocity( const geometry_msgs::Twist::ConstPtr& velocity );

	void OnRosCallback( const std_msgs::String::ConstPtr& msg );

private:

	constexpr static unsigned int REFRESH_RATE_HZ = 100;

	ros::NodeHandle 			m_rosNodeHandle;

	ros::Subscriber				m_llcmdSubscriber;

	ros::Subscriber				m_VelocitySubscriber;

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
