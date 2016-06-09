#include <srslib_framework/robotics/robot/Chuck.hpp>

#include <srsnode_motion/StatePe.hpp>
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
    StatePe<TYPE> state(stateVector);

    state.velocity = reinterpret_cast<CmdVelocity<>*>(command)->velocity;

    double v = state.velocity.linear;
    double w = state.velocity.angular;

    // Check for the special case in which omega is 0 (the robot is moving straight)
    if (abs(w) > ANGULAR_VELOCITY_EPSILON)
    {
        double r = v / w;

        state.pose = Pose<>(
            state.pose.x + r * sin(state.pose.theta + w * dT) - r * sin(state.pose.theta),
            state.pose.y + r * cos(state.pose.theta) - r * cos(state.pose.theta + w * dT),
            state.pose.theta + w * dT);
    }
    else
    {
        state.pose = Pose<>(
            state.pose.x + v * dT * cos(state.pose.theta),
            state.pose.y + v * dT * sin(state.pose.theta),
            state.pose.theta);
    }

    return state.getVectorForm();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods

} // namespace srs
