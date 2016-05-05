/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef STARGAZER_DRIVER_HPP_
#define STARGAZER_DRIVER_HPP_

#include <ros/ros.h>
#include "StarGazer.h"

namespace srs {

class StarGazerDriver
{
public:
	StarGazerDriver( );

	~StarGazerDriver( )
	{

	}

	void run( );

private:

	void OdometryCallback( int tagID, float x, float y, float z, float angle );

private:

	constexpr static unsigned int REFRESH_RATE_HZ = 10;

    ros::NodeHandle rosNodeHandle_;

    ros::Publisher rosApsPublisher;

    StarGazer starGazer_;
};

} // namespace srs

#endif  // STARGAZER_DRIVER_
