#include <srsnode_motion/PositionUkf.hpp>

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

    cv::Mat R = Math::zeros(X.col(1));

    for (unsigned int i = 0; i < X.cols; ++i)
    {
        double weight = W.at<BaseType>(i);
        R += weight * X.col(i);

        double theta = X.at<BaseType>(i, StatePe<>::STATE_THETA);
        c +=  weight * cos(theta);
        s +=  weight * sin(theta);
    }

//    BaseType at = atan2(s / X.cols, c / X.cols);
//    R.at<BaseType>(StatePe<>::STATE_THETA) = at;

    return R;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
cv::Mat PositionUkf::residual(const cv::Mat A, const cv::Mat B)
{
    cv::Mat R = A - B;
//    R.at<BaseType>(StatePe<>::STATE_THETA) = Math::normalizeAngleRad(
//        R.at<BaseType>(StatePe<>::STATE_THETA));

    return R;
}

} // namespace srs
