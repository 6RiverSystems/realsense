#include <srslib_framework/math/Math.hpp>
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

    state.v = reinterpret_cast<CmdVelocity<>*>(command)->velocity.linear;
    state.omega = reinterpret_cast<CmdVelocity<>*>(command)->velocity.angular;

    // Check for the special case in which omega is 0 (the robot is moving straight)
    if (abs(state.omega) > ANGULAR_VELOCITY_EPSILON)
    {
        double r = state.v / state.omega;

        state.x = state.x + r * sin(state.theta + state.omega * dT) - r * sin(state.theta);
        state.y = state.y + r * cos(state.theta) - r * cos(state.theta + state.omega * dT);
        state.theta = state.theta + state.omega * dT;
    }
    else
    {
        state.x = state.x + state.v * dT * cos(state.theta);
        state.y = state.y + state.v * dT * sin(state.theta);
        // theta does not change
    }

    return state.getVectorForm();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods

} // namespace srs
