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
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <dynamic_reconfigure/server.h>
#include <srsnode_odometry/RobotSetupConfig.h>
#include <srslib_framework/OdometryRPM.h>
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
    void readParams();

    void CalculateRobotPose( const srslib_framework::OdometryRPM::ConstPtr& wheelRPM );

    void GetRawOdometryVelocity(const float leftWheelCount, const float rightWheelCount, double& v, double& w);

    void TransformVeclocityToRPM(const geometry_msgs::Twist::ConstPtr& velocity);

    void ResetOdomPose( const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& resetMsg );

    geometry_msgs::Twist getEstimatedRobotVel(double reported_linear_vel, double reported_angular_vel, double reported_time);

    double forwardEstimateVelocity(double old, double cmd, double accel, double dt);

    void pingCallback(const ros::TimerEvent& event);

    void cfgCallback(srsnode_odometry::RobotSetupConfig &config, uint32_t level);

    static constexpr double REFRESH_RATE_HZ = 100;

    static constexpr double PING_HZ = 10;

    static constexpr double MAX_ALLOWED_PING_DELAY = 0.5; // 50% of the duty cycle

    static constexpr double MAX_ALLOWED_ODOM_DELAY = 0.02; // 20ms

	static constexpr auto ODOMETRY_RPM_RAW_TOPIC = "/internal/sensors/odometry/rpm/raw";

	static constexpr auto ODOMETRY_RAW_VELOCITY_TOPIC = "/internal/sensors/odometry/velocity/cmd";

	static constexpr auto ODOMETRY_RPM_COMMAND_TOPIC = "/internal/sensors/odometry/rpm/cmd";

	static constexpr auto ODOMETRY_OUTPUT_TOPIC = "/internal/sensors/odometry/velocity/pose";

	static constexpr auto ODOMETRY_ESTIMATE_OUTPUT_TOPIC = "/internal/sensors/odometry/velocity/estimate";

	static constexpr auto INITIAL_POSE_TOPIC = "/request/odometry/initial_pose";

	static constexpr auto PING_COMMAND_TOPIC = "/internal/state/ping";

    ros::NodeHandle nodeHandle_;

    Pose<> pose_;

	ros::Time lastPoseTime_;

    ros::Timer pingTimer_;

	tf::TransformBroadcaster broadcaster_;

	dynamic_reconfigure::Server<srsnode_odometry::RobotSetupConfig> configServer_;

	ros::Subscriber rawOdometryRPMSub_;

	ros::Subscriber rawVelocityCmdSub_;

	ros::Subscriber resetPoseSub_;

	ros::Publisher rpmVelocityCmdPub_;

	ros::Publisher odometryPosePub_;

	ros::Publisher odometryPoseEstimatePub_;

	ros::Publisher pingPub_;

	double wheelbaseLength_;

	double leftWheelRadius_;

	double rightWheelRadius_;

  double  cmd_vel_timeout_ = 0.5;

  double linear_acceleration_rate_ = 0.7; // m/s^2 (from the firmware)

  double angular_acceleration_rate_ = 2.63; // rad/s^2 (from the firmware)

  double velocity_loop_delays_ = 0.1;  // Time lag between sending command and getting back velocities (from measurements)

  boost::mutex cmd_vel_mutex_;

  geometry_msgs::Twist cmd_vel_;

  double cmd_vel_time_ = 0;
};

} // namespace srs

#endif  // ODOMETRY_POSITION_ESTIMATOR_HPP_
