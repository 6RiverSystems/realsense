#include <srslib_framework/robotics/robot/Chuck.hpp>

#include <srsnode_motion/CmdVelocity.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Constant initialization

// Covariance vector of the robot process
template<int TYPE>
const cv::Mat Robot<TYPE>::Q = (
    cv::Mat_<Robot<TYPE>::BaseType>(1, STATIC_UKF_STATE_VECTOR_SIZE) <<
        Robot<TYPE>::ERROR_LOCATION * Robot<TYPE>::ERROR_LOCATION, // [m^2]
        Robot<TYPE>::ERROR_LOCATION * Robot<TYPE>::ERROR_LOCATION, // [m^2]
        Robot<TYPE>::ERROR_HEADING * Robot<TYPE>::ERROR_HEADING, // [rad^2]
        Robot<TYPE>::ERROR_LINEAR_VELOCITY * Robot<TYPE>::ERROR_LINEAR_VELOCITY, // [m^2/s^2]
        Robot<TYPE>::ERROR_ANGULAR_VELOCITY * Robot<TYPE>::ERROR_ANGULAR_VELOCITY // [m^2/s^2]
);

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
template<int TYPE>
cv::Mat Robot<TYPE>::FB(
    const cv::Mat stateVector,
    Command<STATIC_UKF_COMMAND_VECTOR_SIZE, TYPE>* const command,
    BaseType dT)
{
    StatePe<TYPE> oldState(stateVector);
    oldState.velocity = reinterpret_cast<CmdVelocity<>*>(command)->velocity;

    StatePe<TYPE> newState;
    Robot<TYPE>::kinematics(oldState, dT, newState);

    return newState.getVectorForm();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
template<int TYPE>
void Robot<TYPE>::kinematics(StatePe<TYPE> sT0, BaseType dT, StatePe<TYPE>& sT1)
{
    double v = sT0.velocity.linear;
    double w = sT0.velocity.angular;

    // Check for the special case in which omega is 0 (the robot is moving straight)
    if (abs(w) > ANGULAR_VELOCITY_EPSILON)
    {
        double r = v / w;

        sT1.pose = Pose<>(
            sT0.pose.x + r * sin(sT0.pose.theta + w * dT) - r * sin(sT0.pose.theta),
            sT0.pose.y + r * cos(sT0.pose.theta) - r * cos(sT0.pose.theta + w * dT),
            sT0.pose.theta + w * dT);
    }
    else
    {
        sT1.pose = Pose<>(
            sT0.pose.x + v * dT * cos(sT0.pose.theta),
            sT0.pose.y + v * dT * sin(sT0.pose.theta),
            sT0.pose.theta);
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods

} // namespace srs
