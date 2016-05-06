/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */

#ifndef STARGAZER_HPP_
#define STARGAZER_HPP_

#include <ros/ros.h>
#include <StarGazerDriver.h>

namespace srs
{

class StarGazer
{
public:
	StarGazer( const std::string& strSerialPort, const std::string& strApsTopic );

	virtual ~StarGazer( );

	void run( );

private:

	void OdometryCallback( int tagID, float x, float y, float z, float angle );

private:

	constexpr static unsigned int REFRESH_RATE_HZ = 10;

	ros::NodeHandle rosNodeHandle_;

	ros::Publisher rosApsPublisher;

	StarGazerDriver starGazerDriver_;
};

} // namespace srs

#endif  // STARGAZER_HPP_
