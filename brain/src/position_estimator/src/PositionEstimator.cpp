#include "PositionEstimator.hpp"

#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <boost/assert.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
PositionEstimator::PositionEstimator() :
    ukf_(ALPHA, BETA, robot_, 1 / REFRESH_RATE_HZ),
    rosNodeHandle_(),
    currentCommand_(nullptr)
{
    sensors_.clear();

    currentState_ = PEState<>(0.0, 0.0, 0.0);
    currentCovariance_ = cv::Mat::zeros(STATIC_UKF_STATE_VECTOR_SIZE, STATIC_UKF_STATE_VECTOR_SIZE, CV_64F);

    ukf_.reset(currentState_.getStateVector(), currentCovariance_);

    rosSubscriberCmdVel_ = rosNodeHandle_.subscribe("/cmd_vel", 100,
        &PositionEstimator::cbCmdVelReceived, this);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
PositionEstimator::~PositionEstimator()
{
    sensors_.clear();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void PositionEstimator::addSensor(const RosSensor* newSensor)
{
    BOOST_ASSERT_MSG(newSensor != nullptr, "Expected sensor pointer not null");

    ROS_INFO_STREAM("Registering sensor: " << newSensor->getName());

    sensors_.push_back(newSensor);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void PositionEstimator::run()
{
    ros::Rate refreshRate(REFRESH_RATE_HZ);

    while (ros::ok())
    {
        ukf_.run(currentCommand_);

        if (currentCommand_)
        {
            delete currentCommand_;
            currentCommand_ = nullptr;
        }

        refreshRate.sleep();
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods

////////////////////////////////////////////////////////////////////////////////////////////////////
void PositionEstimator::cbCmdVelReceived(geometry_msgs::TwistConstPtr message)
{
    currentCommand_ = new VelCmd<>(message->linear.x, message->angular.z);
}

} // namespace srs
