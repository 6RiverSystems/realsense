/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */

#ifndef ODOMETRY_POSITION_ESTIMATOR_HPP_
#define ODOMETRY_POSITION_ESTIMATOR_HPP_

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <dynamic_reconfigure/server.h>
#include <srsnode_odometry/RobotSetupConfig.h>
#include <srslib_framework/Odometry.h>
#include <srslib_framework/ros/RosTap.hpp>
#include <srslib_framework/robotics/Pose.hpp>

namespace srs {

class OdometryPositionEstimator
{
public:
    OdometryPositionEstimator(std::string nodeName);

    virtual ~OdometryPositionEstimator();

    void run();

    void connect();

    void disconnect();

private:

    void RawOdomCountToVelocity( const srslib_framework::Odometry::ConstPtr& encoderCount );

    void GetRawOdometryVelocity(const int32_t leftWheelCount, const int32_t rightWheelCount, double timeInterval, double& v, double& w);

    void ResetOdomPose( const geometry_msgs::PoseStamped::ConstPtr& resetMsg );

    void pingCallback(const ros::TimerEvent& event);

    void cfgCallback(srsnode_odometry::RobotSetupConfig &config, uint32_t level);

    static constexpr double REFRESH_RATE_HZ = 100;

    static constexpr double PING_HZ = 10;

    static constexpr double MAX_ALLOWED_PING_DELAY = 0.5; // 50% of the duty cycle

	static constexpr auto ODOMETRY_RAW_COUNT_TOPIC = "/internal/sensors/odometry/count";

	static constexpr auto ODOMETRY_OUTPUT_TOPIC = "/internal/sensors/odometry/velocity";

	static constexpr auto RESET_ODOMETRY_POSE_TOPIC = "/odom_init_pose";

    ros::NodeHandle nodeHandle_;

    Pose<> pose_;

    ros::Timer pingTimer_;

	tf::TransformBroadcaster broadcaster_;

	dynamic_reconfigure::Server<srsnode_odometry::RobotSetupConfig> configServer_;

	ros::Subscriber rawOdometryCountSub_;

	ros::Subscriber resetPoseSub_;

	ros::Publisher odometryPosePub_;

	ros::Publisher pingPub_;

	int motorCountPerRev_;

	double gearboxRatio_;

	double wheelbaseLength_;

	double leftWheelRadius_;

	double rightWheelRadius_;

};

} // namespace srs

#endif  // ODOMETRY_POSITION_ESTIMATOR_HPP_
