#include "PositionEstimator.hpp"

#include <ros/ros.h>
#include <boost/assert.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Constant initialization

const unsigned int PositionEstimator::REFRESH_RATE_HZ = 1;
const unsigned int PositionEstimator::STATE_VECTOR_SIZE = STATIC_STATE_VECTOR_SIZE;
const double PositionEstimator::ALPHA = 1.0;
const double PositionEstimator::BETA = 0.0;

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
PositionEstimator::PositionEstimator() :
    ukf_(ALPHA, BETA, robot_, 1 / REFRESH_RATE_HZ)
{
    sensors_.clear();

    //ukf_.reset(state_, covariance_);
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

    vector<Measurement<>> measurements;
    while (ros::ok())
    {
        measurements.clear();
        for (auto sensor : sensors_)
        {
//            measurements.push_back(sensor->getCurrentData());
        }


        refreshRate.sleep();
    }
}

} // namespace srs
