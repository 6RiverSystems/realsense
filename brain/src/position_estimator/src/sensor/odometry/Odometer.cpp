#include <framework/Math.hpp>

#include "PEState.hpp"

namespace srs {

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
cv::Mat Odometer<STATE_SIZE, TYPE>::transformWithH(const cv::Mat stateVector)
{
    PEState<TYPE> state = PEState<TYPE>(stateVector);

    // Transfer the value that the odometer care about to the new state
    cv::Mat result = Math::zeros(stateVector);
    result.at<BaseType>(PEState<TYPE>::STATE_V) = state.v;
    result.at<BaseType>(PEState<TYPE>::STATE_OMEGA) = state.omega;

    return result;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
template<unsigned int STATE_SIZE, int TYPE>
cv::Mat Odometer<STATE_SIZE, TYPE>::getCurrentData()
{
    // Transfer the value that the odometer care about to the new state
    PEState<TYPE> state = PEState<TYPE>();
    state.v = 0.5 * (currentData_.left + currentData_.right);
    state.omega = wheelsRatio_ * (currentData_.right - currentData_.left);

    return state.getStateVector();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods

} // namespace srs
