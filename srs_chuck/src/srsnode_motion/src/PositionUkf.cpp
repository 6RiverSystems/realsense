#include <srsnode_motion/PositionUkf.hpp>

#include <srslib_framework/math/AngleMath.hpp>
#include <srslib_framework/math/MatrixMath.hpp>

#include <srsnode_motion/StatePe.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
// protected methods

//////////////////////////////////////////////////////////////////////////////////////////////////////
cv::Mat PositionUkf::addWeighted(const cv::Mat W, const cv::Mat X)
{
    double c = 0.0;
    double s = 0.0;

    cv::Mat R = MatrixMath::zeros(X.col(1));

    for (unsigned int i = 0; i < X.cols; ++i)
    {
        BaseType weight = W.at<BaseType>(i);
        R += weight * X.col(i);

        BaseType theta = X.at<BaseType>(StatePe<>::STATE_THETA, i);
        c +=  weight * cos(theta);
        s +=  weight * sin(theta);
    }

    R.at<BaseType>(StatePe<>::STATE_THETA) = atan2(s, c);

    return R;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
cv::Mat PositionUkf::residual(const cv::Mat A, const cv::Mat B)
{
    cv::Mat R = A - B;
    R.at<BaseType>(StatePe<>::STATE_THETA) = AngleMath::normalizeAngleRad(
        R.at<BaseType>(StatePe<>::STATE_THETA));

    return R;
}

} // namespace srs
