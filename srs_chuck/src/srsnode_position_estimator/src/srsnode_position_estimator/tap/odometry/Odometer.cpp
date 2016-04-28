#include <srslib_framework/math/Math.hpp>

#include <srsnode_position_estimator/StatePe.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Const definition

template<unsigned int STATE_SIZE, int TYPE>
const cv::Mat Odometer<STATE_SIZE, TYPE>::R = (
    cv::Mat_<Odometer<STATE_SIZE, TYPE>::BaseType>(1, STATE_SIZE) <<
        0,
        0,
        0,
        0.1,
        0.1
);

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
template<unsigned int STATE_SIZE, int TYPE>
cv::Mat Odometer<STATE_SIZE, TYPE>::getCurrentData()
{
    // Transfer the value that the odometer care about to the new state
    StatePe<TYPE> state = StatePe<TYPE>();
    state.v = currentData_.linear;
    state.omega = currentData_.angular;

    return state.getVectorForm();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
template<unsigned int STATE_SIZE, int TYPE>
cv::Mat Odometer<STATE_SIZE, TYPE>::transformWithH(const cv::Mat stateVector)
{
    StatePe<TYPE> state = StatePe<TYPE>(stateVector);

    // Transfer the value that the odometer care about to the new state
    cv::Mat result = Math::zeros(stateVector);
    result.at<BaseType>(StatePe<TYPE>::STATE_V, 0) = state.v;
    result.at<BaseType>(StatePe<TYPE>::STATE_OMEGA, 0) = state.omega;

    return result;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods

} // namespace srs