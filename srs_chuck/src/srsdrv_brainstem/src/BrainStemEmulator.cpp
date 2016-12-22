#include <BrainStemEmulator.hpp>

#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

namespace srs {

BrainStemEmulator::BrainStemEmulator( ) :
	m_rosNodeHandle(),
	m_odometryTimer()
{
	CreatePublishers( );

	CreateSubscribers( );
//
//    // Start the odometry timer
//	m_odometryTimer = m_rosNodeHandle.createTimer(ros::Duration(1.0 / ODOMETRY_RATE_HZ),
//        boost::bind(&BrainStemEmulator::PublishOdometry, this, _1));
}

BrainStemEmulator::~BrainStemEmulator( )
{

}

void BrainStemEmulator::CreateSubscribers( )
{
	m_velocitySubscriber = m_rosNodeHandle.subscribe<srslib_framework::OdometryRpm>(VELOCITY_TOPIC, 10,
		std::bind( &BrainStemEmulator::OnChangeVelocity, this, std::placeholders::_1 ) );
}

void BrainStemEmulator::CreatePublishers( )
{
	m_odometryPublisher = m_rosNodeHandle.advertise<srslib_framework::OdometryRpm>(ODOMETRY_TOPIC, 20);
}

void BrainStemEmulator::OnChangeVelocity( const srslib_framework::OdometryRpm::ConstPtr& velocity )
{
	// Change the current velocity
	m_velocity = *velocity;
}

void BrainStemEmulator::PublishOdometry(const ros::TimerEvent& event)
{
	srslib_framework::OdometryRpm msg;

    msg.header.stamp = ros::Time::now( );
    msg.left_wheel_rpm = m_velocity.left_wheel_rpm;
    msg.right_wheel_rpm = m_velocity.right_wheel_rpm;

	m_odometryPublisher.publish(msg);
}

}
