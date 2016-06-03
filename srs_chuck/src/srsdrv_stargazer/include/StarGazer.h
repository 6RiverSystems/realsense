/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */

#ifndef STARGAZER_HPP_
#define STARGAZER_HPP_

#include <ros/ros.h>
#include <StarGazerMessageProcessor.h>
#include <StarGazerFilter.h>
#include <srslib_framework/io/IO.hpp>
#include <srslib_framework/localization/Anchor.hpp>
#include "tf/tf.h"
#include <tf/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>

namespace srs
{

class StarGazer
{
	typedef shared_ptr<tf2_ros::StaticTransformBroadcaster> TransformBroadcasterPtr;

public:
	StarGazer( const std::string& strNodeName, const std::string& strSerialPort,
		const std::string& strApsTopic );

	virtual ~StarGazer( );

	void Run( );

	void OnConnectionChanged( bool bIsConnected );

	void SetOdometryCallback( OdometryCallbackFn callback );

	void HardReset( );

	void Configure( );

	void SetFixedHeight( int height_mm );

	void SetVariableHeight( );

	void AutoCalculateHeight( );

	void Start( );

	void Stop( );

	void PumpMessageProcessor( );

	void ReadCallback( std::string strType, std::string strValue );

private:

// Message Processing

	void OnVersion( std::vector<std::string> vecParams );

	void OdometryCallback( int tagID, float x, float y, float z, float angle );

	void LoadAnchors( );

	std::string GetAnchorFrame( int tagID ) const;

private:

	constexpr static unsigned int 		REFRESH_RATE_HZ = 20;

	ros::NodeHandle						m_rosNodeHandle;

	ros::Publisher						m_rosApsPublisher;

	std::shared_ptr<IO>					m_pSerialIO;

	StarGazerMessageProcessor			m_messageProcessor;

	StarGazerFilter						m_filter;

	std::string							m_strAnchorTargetFrame;

	std::string							m_strPoseTargetFrame;

	ros::Duration						m_sleeper;

	std::map<int, TransformBroadcasterPtr>	m_mapTransformSenders;

	std::map<int, tf::Transform>		m_mapTransforms;

	tf::TransformListener				m_listener;

	tf::StampedTransform				m_baseFootprintTransform;

	tf::Transform						m_rotationTransform;

};

} // namespace srs

#endif  // STARGAZER_HPP_
