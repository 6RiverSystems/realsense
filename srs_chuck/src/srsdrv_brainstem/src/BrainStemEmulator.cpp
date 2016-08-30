#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <srsdrv_brainstem/BrainStemEmulator.h>

namespace srs {

BrainStemEmulator::BrainStemEmulator( ) :
	m_rosNodeHandle(),
	m_odometryTimer()
{
	CreatePublishers( );

	CreateSubscribers( );

    // Start the odometry timer
	m_odometryTimer = m_rosNodeHandle.createTimer(ros::Duration(1.0 / ODOMETRY_RATE_HZ),
        boost::bind(&BrainStemEmulator::PublishOdometry, this, _1));
}

BrainStemEmulator::~BrainStemEmulator( )
{

}

void BrainStemEmulator::CreateSubscribers( )
{
	m_velocitySubscriber = m_rosNodeHandle.subscribe<geometry_msgs::Twist>(VELOCITY_TOPIC, 10,
		std::bind( &BrainStemEmulator::OnChangeVelocity, this, std::placeholders::_1 ) );
}

void BrainStemEmulator::CreatePublishers( )
{
	m_odometryPublisher = m_rosNodeHandle.advertise<geometry_msgs::TwistStamped>(ODOMETRY_TOPIC, 20);
}

void BrainStemEmulator::OnChangeVelocity( const geometry_msgs::Twist::ConstPtr& velocity )
{
	// Change the current velocity
	m_velocity = *velocity;
}

void BrainStemEmulator::PublishOdometry(const ros::TimerEvent& event)
{
    geometry_msgs::TwistStamped msgTwistStamped;

    msgTwistStamped.header.stamp = ros::Time::now( );
    msgTwistStamped.twist.linear.x = m_velocity.linear.x;
    msgTwistStamped.twist.angular.z = m_velocity.angular.z;

	m_odometryPublisher.publish(msgTwistStamped);
}

}
