#include <framework/Math.hpp>

#include <filter/FilterState.hpp>

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
cv::Mat Odometer<STATE_SIZE, TYPE>::transformWithH(const cv::Mat state)
{
    BaseType v = state.at<BaseType>(FilterState<TYPE>::STATE_V);
    BaseType omega = state.at<BaseType>(FilterState<TYPE>::STATE_OMEGA);

    // Transfer the value that the odometer care about to the new state
    cv::Mat result = Math::zeros(state);
    result.at<BaseType>(FilterState<TYPE>::STATE_V) = v;
    result.at<BaseType>(FilterState<TYPE>::STATE_OMEGA) = omega;

    return result;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
template<unsigned int STATE_SIZE, int TYPE>
cv::Mat Odometer<STATE_SIZE, TYPE>::transform2State(Measurement<STATE_SIZE, TYPE>* const measurement)
{
    BaseType left = reinterpret_cast<Odometry<STATE_SIZE, TYPE>*>(measurement)->left;
    BaseType right = reinterpret_cast<Odometry<STATE_SIZE, TYPE>*>(measurement)->right;

    BaseType v = BaseType(0.5) * (left + right);
    BaseType omega = wheelsRatio_ * (right - left);

    // Transfer the value that the odometer care about to the new state
    cv::Mat result = cv::Mat::zeros(STATE_SIZE, 1, TYPE);
    result.at<BaseType>(FilterState<TYPE>::STATE_V) = v;
    result.at<BaseType>(FilterState<TYPE>::STATE_OMEGA) = omega;

    return result;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods

} // namespace srs
