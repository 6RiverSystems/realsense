/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef MIDBRAIN_REFLEXES_HPP_
#define MIDBRAIN_REFLEXES_HPP_

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <srsnode_midbrain/ObstacleDetector.hpp>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/TwistStamped.h>
#include <srslib_framework/MsgOperationalState.h>
#include <srslib_framework/MsgPose.h>
#include <srslib_framework/MsgSolution.h>
#include <dynamic_reconfigure/server.h>
#include <srsnode_midbrain/ReflexesConfig.h>

namespace srs
{

class Reflexes
{
public:
	Reflexes( ros::NodeHandle& nodeHandle );
	virtual ~Reflexes( );

// Configuration OptionsPathChanged

	void Enable( bool enableDepthDetection );

	void SetObjectThreshold( uint32_t depthThreshold );

// Topic Callbacks

	void OnOperationalStateChanged( const srslib_framework::MsgOperationalState::ConstPtr& operationalState );

	void OnCommandVelocityChanged( const geometry_msgs::Twist::ConstPtr& velocity );

	void OnOdomVelocityChanged( const geometry_msgs::TwistStamped::ConstPtr& velocity );

	void OnPoseChanged( const srslib_framework::MsgPose::ConstPtr& pose );

	void OnSolutionChanged( const srslib_framework::MsgSolution::ConstPtr& solution );

	void OnLaserScan( const sensor_msgs::LaserScan::ConstPtr& scan );

	void PublishDangerZone( ) const;

private:

    void onConfigChange(srsnode_midbrain::ReflexesConfig& config, uint32_t level);

    void OnObstacleDetected( );

	void CreateSubscribers( );

	void CreatePublishers( );

	static constexpr auto OPERATIONAL_STATE_TOPIC = "/info/operational_state";

	static constexpr auto ODOMETRY_TOPIC = "/internal/sensors/odometry/raw";

	static constexpr auto CMD_VELOCITY_TOPIC = "/internal/drivers/brainstem/cmd_velocity";

	static constexpr auto SOLUTION_TOPIC = "/internal/state/goal/solution";

	static constexpr auto POSE_TOPIC = "/internal/state/robot/pose";

	static constexpr auto DEPTH_SCAN_TOPIC = "/camera/depth/scan";

	static constexpr auto DANGER_ZONE_TOPIC = "/internal/state/reflexes/danger_zone";

	static constexpr auto COMMAND_TOPIC = "/cmd_ll";

	// TODO: Get footprint from robot model (lookup topic, load urdf, etc.)
	static constexpr auto ROBOT_WIDTH = 0.64f;

    dynamic_reconfigure::Server<srsnode_midbrain::ReflexesConfig> m_configServer;

	ros::NodeHandle&						m_nodeHandle;

	bool									m_enableIrDetection;

	bool									m_enableDepthDetection;

	bool	 								m_sendHardStop;

	srslib_framework::MsgOperationalState	m_operationalState;

	ObstacleDetector 						m_obstacleDetector;

	ros::Subscriber							m_operationalStateSubscriber;

	ros::Subscriber							m_irScanSubscriber;

	ros::Subscriber							m_depthScanSubscriber;

	ros::Subscriber							m_commandVelocitySubscriber;

	ros::Subscriber							m_odomVelocitySubscriber;

	ros::Subscriber							m_poseSubscriber;

	ros::Subscriber							m_solutionSubscriber;

	ros::Publisher							m_commandPublisher;

    ros::Publisher							m_dangerZonePublisher;

};

} /* namespace srs */

#endif /* MIDBRAIN_REFLEXES_HPP_ */
