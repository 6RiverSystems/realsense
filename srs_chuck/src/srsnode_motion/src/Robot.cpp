#include <srslib_framework/math/Math.hpp>
#include <srslib_framework/robotics/robot/Chuck.hpp>

#include <srsnode_motion/StatePe.hpp>
#include <srsnode_motion/CmdVelocity.hpp>

namespace srs {

template<int TYPE>
const cv::Mat Robot<TYPE>::Q = (
    cv::Mat_<Robot<TYPE>::BaseType>(1, STATIC_UKF_STATE_VECTOR_SIZE) <<
        0.01, // [m]
        0.01, // [m]
        Math::deg2rad<double>(1.0), // [rad]
        0.006, // [m/s]
        0.006 // [m/s]
);

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
template<int TYPE>
cv::Mat Robot<TYPE>::transformWithAB(const cv::Mat stateVector,
    Command<STATIC_UKF_COMMAND_VECTOR_SIZE, TYPE>* const command,
    BaseType dT)
{
    StatePe<TYPE> state(stateVector);

    // x = x + v * DT * cos(theta);
    // y = y + v * DT * sin(theta);
    // theta = theta + omega * DT;
    state.x = state.x + state.v * dT * cos(state.theta);
    state.y = state.y + state.v * dT * sin(state.theta);
    state.theta = state.theta + state.omega * dT;

    // if command.valid
    //     v = command.velocity.linear;
    //     omega = command.velocity.angular;
    // end
    if (command)
    {
        state.v = reinterpret_cast<CmdVelocity<>*>(command)->velocity.linear;
        state.omega = reinterpret_cast<CmdVelocity<>*>(command)->velocity.angular;
    }

    return state.getVectorForm();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods

} // namespace srs
