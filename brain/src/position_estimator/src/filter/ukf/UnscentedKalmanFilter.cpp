#include <iostream>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
template<int TYPE>
UnscentedKalmanFilter<TYPE>::UnscentedKalmanFilter(unsigned int n, double alpha, double beta) :
    n_(n),
    alpha_(alpha),
    beta_(beta),
    kappa_(0.0),
    lambda_(0.0)
{
    initializeWeights();
    reset();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
template<int TYPE>
UnscentedKalmanFilter<TYPE>::~UnscentedKalmanFilter()
{
}

////////////////////////////////////////////////////////////////////////////////////////////////////
template<int TYPE>
void UnscentedKalmanFilter<TYPE>::reset()
{
    state_ = cv::Mat::zeros(1, n_, CV_64F);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods

////////////////////////////////////////////////////////////////////////////////////////////////////
template<int TYPE>
void UnscentedKalmanFilter<TYPE>::initializeWeights()
{
    // o.kappa = 3 - o.n;
    // o.lambda = o.alpha ^ 2 * (o.n + o.kappa) - o.n;
    kappa_ = 3.0 - n_;
    lambda_ = pow(alpha_, 2.0) * (n_ + kappa_) - n_;

    // o.WM = zeros(2 * o.n + 1, 1);
    // o.WC = zeros(2 * o.n + 1, 1);

    // o.WM(1) = o.lambda / (o.n + o.lambda);
    // o.WC(1) = o.lambda / (o.n + o.lambda) + (1 - o.alpha ^ 2 + o.beta);

    // for j = 2 : 2 * o.n + 1
    //     o.WM(j) = 1 / (2 * (o.n + o.lambda));
    //     o.WC(j) = o.WM(j);
    // end
    unsigned int size = 2 * n_ + 1;
    double value = 1.0 / (2.0 * (n_ + lambda_));
    WM_ = cv::Mat(size, 1, CV_64F, value);
    WC_ = cv::Mat(size, 1, CV_64F, value);

    WM_.at<double>(0) = lambda_ / (n_ + lambda_);
    WC_.at<double>(0) = lambda_ / (n_ + lambda_) + (1.0 - pow(alpha_, 2.0) + beta_);

    std::cout << WM_ << std::endl;

    // o.c = o.n + o.lambda;
    c_ = n_ + lambda_;
}

} // namespace srs
