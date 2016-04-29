#include <srslib_framework/math/Math.hpp>

#include <srsnode_position_estimator/StatePe.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Const definition

template<unsigned int STATE_SIZE, int TYPE>
const cv::Mat ApsSensor<STATE_SIZE, TYPE>::R = (
    cv::Mat_<BaseType>(1, STATE_SIZE) <<
        0,
        0,
        0,
        0.001,
        0.001
);

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
template<unsigned int STATE_SIZE, int TYPE>
cv::Mat ApsSensor<STATE_SIZE, TYPE>::getCurrentData()
{
    // Transfer the value that the odometer care about to the new state
    StatePe<TYPE> state = StatePe<TYPE>();
    state.x = currentData_.x;
    state.y = currentData_.y;
    state.theta = currentData_.theta;

    return state.getVectorForm();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
template<unsigned int STATE_SIZE, int TYPE>
cv::Mat ApsSensor<STATE_SIZE, TYPE>::transformWithH(const cv::Mat stateVector)
{
    StatePe<TYPE> state = StatePe<TYPE>(stateVector);

    // Transfer the value that the odometer care about to the new state
    cv::Mat result = Math::zeros(stateVector);
    result.at<BaseType>(StatePe<TYPE>::STATE_X, 0) = state.x;
    result.at<BaseType>(StatePe<TYPE>::STATE_Y, 0) = state.y;
    result.at<BaseType>(StatePe<TYPE>::STATE_THETA, 0) = state.theta;

    return result;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods

} // namespace srs
