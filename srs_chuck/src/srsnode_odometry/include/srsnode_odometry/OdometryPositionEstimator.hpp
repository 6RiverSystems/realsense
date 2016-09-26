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
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <srslib_framework/ros/tap/RosTap.hpp>
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

    void RawOdometryVelocity( const geometry_msgs::TwistStamped::ConstPtr& estimatedVelocity );

    void pingCallback(const ros::TimerEvent& event);

    static constexpr double REFRESH_RATE_HZ = 100;

    static constexpr double PING_HZ = 10;

    static constexpr double MAX_ALLOWED_PING_DELAY = 0.5; // 50% of the duty cycle

	static constexpr auto ODOMETRY_RAW_TOPIC = "/internal/sensors/odometry/raw";

	static constexpr auto ODOMETRY_TOPIC = "/internal/sensors/odometry/velocity";

    ros::NodeHandle nodeHandle_;

    geometry_msgs::Twist twist_;

    Pose<> pose_;

    ros::Timer pingTimer_;

	tf::TransformBroadcaster broadcaster_;

	ros::Subscriber rawOdometrySub_;

	ros::Publisher odometryPub_;

	ros::Publisher pingPub_;

};

} // namespace srs

#endif  // ODOMETRY_POSITION_ESTIMATOR_HPP_
