#include <framework/Math.hpp>
#include <framework/Utils.hpp>

#include <filter/FilterState.hpp>

namespace srs {

template<unsigned int STATE_SIZE, int TYPE>
const cv::Mat Robot<STATE_SIZE, TYPE>::Q = (
    cv::Mat_<Robot<STATE_SIZE, TYPE>::BaseType>(1, STATE_SIZE) <<
        0.0001,
        0.0001,
        Math::deg2rad(0.01),
        0,
        0
);

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
template<unsigned int STATE_SIZE, int TYPE>
cv::Mat Robot<STATE_SIZE, TYPE>::transformWithAB(const cv::Mat state,
    Command<TYPE>* const command,
    BaseType dT)
{
    // x = x + v * DT * cos(theta);
    // y = y + v * DT * sin(theta);
    // theta = theta + omega * DT;
    BaseType x = state.at<BaseType>(FilterState<TYPE>::STATE_X);
    BaseType y = state.at<BaseType>(FilterState<TYPE>::STATE_Y);
    BaseType theta = state.at<BaseType>(FilterState<TYPE>::STATE_THETA);
    BaseType v = state.at<BaseType>(FilterState<TYPE>::STATE_V);
    BaseType omega = state.at<BaseType>(FilterState<TYPE>::STATE_OMEGA);

    x = x + v * dT * cos(theta);
    y = y + v * dT * sin(theta);
    theta = theta + omega * dT;

    // if command.valid
    //     v = command.velocity.linear;
    //     omega = command.velocity.angular;
    // end
    if (command)
    {
        v = command->getV();
        omega = command->getOmega();
    }

    // Update the state to the next step
    cv::Mat result = Math::zeros(state);
    result.at<BaseType>(FilterState<TYPE>::STATE_X) = x;
    result.at<BaseType>(FilterState<TYPE>::STATE_Y) = y;
    result.at<BaseType>(FilterState<TYPE>::STATE_THETA) = theta;
    result.at<BaseType>(FilterState<TYPE>::STATE_V) = v;
    result.at<BaseType>(FilterState<TYPE>::STATE_OMEGA) = omega;

    return result;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods

} // namespace srs
