/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */

#ifndef BRAINSTEM_EMULATOR_HPP_
#define BRAINSTEM_EMULATOR_HPP_

#include <ros/ros.h>

namespace srs
{

class BrainStemEmulator
{

public:

	BrainStemEmulator( );

	virtual ~BrainStemEmulator( );

private:

	void CreateSubscribers( );

	void CreatePublishers( );

	void OnChangeVelocity( const geometry_msgs::Twist::ConstPtr& velocity );

	void PublishOdometry(const ros::TimerEvent& event);

private:

	static constexpr auto ODOMETRY_RATE_HZ = 100;

	static constexpr auto VELOCITY_TOPIC = "/internal/drivers/brainstem/cmd_velocity";

	static constexpr auto ODOMETRY_TOPIC = "/internal/sensors/odometry/emu_raw";

	ros::NodeHandle 			m_rosNodeHandle;

	ros::Timer					m_odometryTimer;

	ros::Subscriber				m_velocitySubscriber;

	ros::Publisher				m_odometryPublisher;

	geometry_msgs::Twist		m_velocity;

};

} // namespace srs

#endif  // BRAINSTEM_EMULATOR_HPP_
