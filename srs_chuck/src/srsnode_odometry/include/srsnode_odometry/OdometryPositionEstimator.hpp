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

    void RawOdometryVelocity( const geometry_msgs::TwistStamped::ConstPtr& estimatedVelocity );

    static constexpr double REFRESH_RATE_HZ = 100;

	static constexpr auto ODOMETRY_RAW_TOPIC = "/internal/sensors/odometry/raw";

	static constexpr auto ODOMETRY_TOPIC = "/internal/sensors/odometry/velocity";

    ros::NodeHandle nodeHandle_;

    geometry_msgs::Twist twist_;

    Pose<> pose_;

	tf::TransformBroadcaster broadcaster_;

	ros::Subscriber rawOdometrySub_;

	ros::Publisher odometryPub_;

	ros::Publisher pingPub_;

};

} // namespace srs

#endif  // ODOMETRY_POSITION_ESTIMATOR_HPP_
