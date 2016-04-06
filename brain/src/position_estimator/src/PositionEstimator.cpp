#include "PositionEstimator.hpp"

#include <ros/ros.h>
#include <boost/assert.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Constant initialization

const unsigned int PositionEstimator::REFRESH_RATE_HZ = 100;
const unsigned int PositionEstimator::STATE_VECTOR_SIZE = 5;
const double PositionEstimator::ALPHA = 1.0;
const double PositionEstimator::BETA = 0.0;

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
PositionEstimator::PositionEstimator(const SensorFrameQueue* queue) :
    SensorReadingHandler(queue),
    ukf_(STATE_VECTOR_SIZE, ALPHA, BETA)
{
    sensors_.clear();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
PositionEstimator::~PositionEstimator()
{
    sensors_.clear();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void PositionEstimator::addSensor(const Sensor* newSensor)
{
    BOOST_ASSERT_MSG(newSensor != nullptr, "Expected sensor pointer not null");

    ROS_INFO_STREAM("Registering sensor: " << newSensor->getName());

    sensors_.push_back(newSensor);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void PositionEstimator::run()
{
    ros::Rate refreshRate(REFRESH_RATE_HZ);

    ukf_.reset();

    while (ros::ok())
    {
        for (const Sensor* sensor : sensors_)
        {
        }

        refreshRate.sleep();
    }
}

} // namespace srs
