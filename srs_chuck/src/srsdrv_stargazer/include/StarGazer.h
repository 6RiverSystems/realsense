/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */

#ifndef STARGAZER_HPP_
#define STARGAZER_HPP_

#include <ros/ros.h>
#include <StarGazerMessageProcessor.h>
#include <srslib_framework/io/IO.hpp>

namespace srs
{

class StarGazer
{
public:
	StarGazer( const std::string& strSerialPort, const std::string& strApsTopic );

	virtual ~StarGazer( );

	void Run( );

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

private:

	constexpr static unsigned int REFRESH_RATE_HZ = 10;

	ros::NodeHandle				m_rosNodeHandle;

	ros::Publisher				m_rosApsPublisher;

	std::shared_ptr<IO>			m_pSerialIO;

	StarGazerMessageProcessor	m_messageProcessor;

};

} // namespace srs

#endif  // STARGAZER_HPP_
