#include <srsdrv_brainstem/hw_message/RawOdometryHandler.hpp>

#include <srslib_framework/Odometry.h>

#include <srslib_framework/math/BasicMath.hpp>
#include <srslib_framework/math/AngleMath.hpp>
#include <srslib_framework/math/TimeMath.hpp>
#include <srslib_framework/math/VelocityMath.hpp>
#include <srslib_framework/ros/message/VelocityMessageFactory.hpp>
#include <srslib_framework/ros/message/ImuMessageFactory.hpp>

namespace srs {

const string RawOdometryHandler::TOPIC_RAW_ODOMETRY = "/internal/sensors/odometry/raw";

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

RawOdometryHandler::RawOdometryHandler() :
    BrainstemMessageHandler(RAW_ODOMETRY_KEY)
{

}

////////////////////////////////////////////////////////////////////////////////////////////////////
void RawOdometryHandler::receiveData(ros::Time currentTime, vector<char>& buffer)
{

}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods

void RawOdometryHandler::publishOdometry(float leftWheelRpm, float rightWheelRpm)
{

}

} // namespace srs
