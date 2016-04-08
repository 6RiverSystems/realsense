#include "PositionEstimator.hpp"

#include <ros/ros.h>
#include <boost/assert.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Constant initialization

const unsigned int PositionEstimator::REFRESH_RATE_HZ = 100;
const unsigned int PositionEstimator::STATE_VECTOR_SIZE = STATIC_STATE_VECTOR_SIZE;
const double PositionEstimator::ALPHA = 1.0;
const double PositionEstimator::BETA = 0.0;

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
PositionEstimator::PositionEstimator(const SensorFrameQueue* queue) :
    SensorReadingHandler(queue),
    state_(),
    ukf_(ALPHA, BETA, robot_)

{
    sensors_.clear();

    //state_ = FilterState<STATIC_STATE_VECTOR_SIZE>();
    covariance_ = cv::Mat::eye(STATE_VECTOR_SIZE, STATE_VECTOR_SIZE, CV_64F);

    ukf_.reset(state_, covariance_);
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

    vector<Measurement> measurements;
    while (ros::ok())
    {
        measurements.clear();
        for (const Sensor* sensor : sensors_)
        {
            measurements.push_back(sensor->getCurrentData());
        }


        refreshRate.sleep();
    }
}

} // namespace srs
