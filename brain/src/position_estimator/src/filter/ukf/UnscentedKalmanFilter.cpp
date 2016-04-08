#include "UnscentedKalmanFilter.hpp"

#include <iostream>
#include <functional>
using namespace std;

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
template<unsigned int STATE_SIZE>
UnscentedKalmanFilter<STATE_SIZE>::UnscentedKalmanFilter(
        double alpha, double beta,
        ProcessModel<>& processModel) :
    alpha_(alpha),
    beta_(beta),
    kappa_(0.0),
    lambda_(0.0),
    processModel_(processModel)
{
    initializeWeights();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
template<unsigned int STATE_SIZE>
UnscentedKalmanFilter<STATE_SIZE>::~UnscentedKalmanFilter()
{
}

////////////////////////////////////////////////////////////////////////////////////////////////////
template<unsigned int STATE_SIZE>
void UnscentedKalmanFilter<STATE_SIZE>::reset(FilterState<STATE_SIZE> stateT0, cv::Mat covarianceT0)
{
    state_ = stateT0.vector;
    covariance_ = covarianceT0;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
template<unsigned int STATE_SIZE>
void UnscentedKalmanFilter<STATE_SIZE>::run(Command<>* const command,
    const std::vector<Measurement> measurements)
{
    prediction(command);
    update();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods

////////////////////////////////////////////////////////////////////////////////////////////////////
template<unsigned int STATE_SIZE>
cv::Mat UnscentedKalmanFilter<STATE_SIZE>::calculateSigmaPoints(cv::Mat M, cv::Mat P)
{
    // A = chol(P)';
    cv::Mat A = Math::cholesky(P);

    // CHI = [zeros(size(M)) A -A];
    cv::Mat chi;
    cv::Mat zM = Math::zeros(M).t();
    cv::hconcat(zM, A, chi);
    cv::hconcat(chi, -A, chi);

    // CHI = sqrt(o.c) * CHI + repmat(M, 1, size(CHI, 2));
    cv::Mat repeatedM;
    cv::repeat(M.t(), 1, chi.cols, repeatedM);
    cv::scaleAdd(chi, sqrt(c_), repeatedM, chi);

    return chi;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
template<unsigned int STATE_SIZE>
void UnscentedKalmanFilter<STATE_SIZE>::initializeWeights()
{
    // o.kappa = 3 - o.n;
    // o.lambda = o.alpha ^ 2 * (o.n + o.kappa) - o.n;
    kappa_ = 3.0 - STATE_SIZE;
    lambda_ = pow(alpha_, 2.0) * (STATE_SIZE + kappa_) - STATE_SIZE;

    // o.WM = zeros(2 * o.n + 1, 1);
    // o.WC = zeros(2 * o.n + 1, 1);

    // o.WM(1) = o.lambda / (o.n + o.lambda);
    // o.WC(1) = o.lambda / (o.n + o.lambda) + (1 - o.alpha ^ 2 + o.beta);

    // for j = 2 : 2 * o.n + 1
    //     o.WM(j) = 1 / (2 * (o.n + o.lambda));
    //     o.WC(j) = o.WM(j);
    // end
    unsigned int size = 2 * STATE_SIZE + 1;
    double value = 1.0 / (2.0 * (STATE_SIZE + lambda_));
    WM_ = cv::Mat(size, 1, CV_64F, value);
    WC_ = cv::Mat(size, 1, CV_64F, value);

    WM_.at<double>(0) = lambda_ / (STATE_SIZE + lambda_);
    WC_.at<double>(0) = WM_.at<double>(0) + (1.0 - pow(alpha_, 2.0) + beta_);

    // o.c = o.n + o.lambda;
    c_ = STATE_SIZE + lambda_;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
template<unsigned int STATE_SIZE>
void UnscentedKalmanFilter<STATE_SIZE>::prediction(Command<>* const command)
{
    // Calculate the sigma points
    //
    // CHI = o.calculateSigmaPoints(XX, P);
    cv::Mat chi = calculateSigmaPoints(state_, covariance_);

    // Pass the sigma points through g()
    //
    // Y = zeros(size(CHI));
    // for i = 1:size(CHI, 2);
    //     if ~isempty(gParams)
    //         Y(:, i) = gFunction(CHI(:, i), gParams);
    //     else
    //         Y(:, i) = gFunction(CHI(:, i));
    //     end
    // end
    cv::Mat Y = Math::zeros(chi);
    for (unsigned int i = 0; i < chi.cols; ++i)
    {
        cv::Mat newState = processModel_.transform(chi.col(i), command, 1.0);

        cout << newState << endl;

        Y.col(i) = newState;
    }

    cout << Y << endl;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
template<unsigned int STATE_SIZE>
void UnscentedKalmanFilter<STATE_SIZE>::unscentedTransform(
        const cv::Mat XX, const cv::Mat Y, const cv::Mat chi,
        cv::Mat& ybar, cv::Mat& S, cv::Mat& C)
{
    // Calculate the predicted mean
    //
    // ybar = zeros(size(Y, 1), 1);
    // for i = 1 : size(CHI, 2)
    //     ybar = ybar + o.WM(i) * Y(:, i);
    // end
    ybar = Math::zeros(Y);

    for (unsigned int i = 0; i < chi.cols; ++i)
    {
        ybar += WM_.at<double>(i) * Y.col(i);
    }

    // Calculate the predicted covariance matrix
    //
    // S  = zeros(size(Y, 1), size(Y, 1));
    // C  = zeros(size(XX, 1), size(Y, 1));
    // for i = 1 : size(CHI, 2)
    //     S = S + o.WC(i) * (Y(:, i) - ybar) * (Y(:, i) - ybar)';
    //     C = C + o.WC(i) * (CHI(1:size(XX, 1), i) - XX) * (Y(:, i) - ybar)';
    // end
    S.setTo(0);
    C.setTo(0);

    cv::Mat tempS = Math::zeros(S);

    for (unsigned int i = 0; i < chi.cols; ++i)
    {
        cv::mulTransposed(Y.col(i), tempS, true, ybar, WC_.at<double>(i));
        S += tempS;
    }

    cout << "S: " << S << endl;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
template<unsigned int STATE_SIZE>
void UnscentedKalmanFilter<STATE_SIZE>::update()
{
}

} // namespace srs
