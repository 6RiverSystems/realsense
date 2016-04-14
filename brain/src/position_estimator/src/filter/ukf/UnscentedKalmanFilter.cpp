#include "UnscentedKalmanFilter.hpp"

#include <iostream>
#include <functional>

#include <framework/Utils.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
template<unsigned int STATE_SIZE, int TYPE>
UnscentedKalmanFilter<STATE_SIZE, TYPE>::UnscentedKalmanFilter(
    BaseType alpha, BaseType beta,
    Process<STATE_SIZE, TYPE>& process,
    BaseType dT) :
        alpha_(alpha),
        beta_(beta),
        kappa_(BaseType()),
        lambda_(BaseType()),
        process_(process),
        dT_(dT)
{
    initializeWeights();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
template<unsigned int STATE_SIZE, int TYPE>
UnscentedKalmanFilter<STATE_SIZE, TYPE>::~UnscentedKalmanFilter()
{
}

////////////////////////////////////////////////////////////////////////////////////////////////////
template<unsigned int STATE_SIZE, int TYPE>
void UnscentedKalmanFilter<STATE_SIZE, TYPE>::reset(
    FilterState<TYPE> stateT0, cv::Mat covarianceT0)
{
    // Initialize the state of the filter with an adapted
    // version of the provided initial state and covariance
    stateT0.vector.convertTo(state_, TYPE);
    covarianceT0.convertTo(covariance_, TYPE);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
template<unsigned int STATE_SIZE, int TYPE>
void UnscentedKalmanFilter<STATE_SIZE, TYPE>::run(Command<TYPE>* const command,
    const vector<Measurement<STATE_SIZE, TYPE>*> measurements)
{
    predict(command);
    update(measurements);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods

////////////////////////////////////////////////////////////////////////////////////////////////////
template<unsigned int STATE_SIZE, int TYPE>
cv::Mat UnscentedKalmanFilter<STATE_SIZE, TYPE>::calculateSigmaPoints(cv::Mat X, cv::Mat P)
{
    // A = chol(P)';
    cv::Mat A = Math::cholesky(P);

    // CHI = [zeros(size(X)) A -A];
    cv::Mat CHI;
    cv::Mat zM = Math::zeros(X);
    cv::Mat array[] = {zM, A, -A};
    cv::hconcat(array, 3, CHI);

    // CHI = o.c * CHI + repmat(X, 1, size(CHI, 2));
    cv::Mat repeatedX;
    cv::repeat(X, 1, CHI.cols, repeatedX);

    cv::scaleAdd(CHI, c_, repeatedX, CHI);

    return CHI;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
template<unsigned int STATE_SIZE, int TYPE>
void UnscentedKalmanFilter<STATE_SIZE, TYPE>::checkDiagonalUnderflow(cv::Mat& S)
{
    Math::checkDiagonal(S, UNDERFLOW_THRESHOLD, UNDERFLOW_THRESHOLD);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
template<unsigned int STATE_SIZE, int TYPE>
void UnscentedKalmanFilter<STATE_SIZE, TYPE>::initializeWeights()
{
    // o.kappa = 3 - o.n;
    // o.lambda = o.alpha ^ 2 * (o.n + o.kappa) - o.n;
    kappa_ = BaseType(3.0) - STATE_SIZE;
    lambda_ = pow(alpha_, BaseType(2.0)) * (STATE_SIZE + kappa_) - STATE_SIZE;

    // o.WM = zeros(2 * o.n + 1, 1);
    // o.WC = zeros(2 * o.n + 1, 1);

    // o.WM(1) = o.lambda / (o.n + o.lambda);
    // o.WC(1) = o.lambda / (o.n + o.lambda) + (1 - o.alpha ^ 2 + o.beta);

    // for j = 2 : 2 * o.n + 1
    //     o.WM(j) = 1 / (2 * (o.n + o.lambda));
    //     o.WC(j) = o.WM(j);
    // end
    unsigned int size = 2 * STATE_SIZE + 1;
    BaseType value = BaseType(1.0) / (BaseType(2.0) * (STATE_SIZE + lambda_));
    WM_ = cv::Mat(size, 1, TYPE, value);
    WC_ = cv::Mat(size, 1, TYPE, value);

    WM_.at<BaseType>(0) = lambda_ / (STATE_SIZE + lambda_);
    WC_.at<BaseType>(0) = WM_.at<BaseType>(0) + (BaseType(1.0) - pow(alpha_, 2.0) + beta_);

    // o.c = o.n + o.lambda;
    c_ = sqrt(STATE_SIZE + lambda_);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
template<unsigned int STATE_SIZE, int TYPE>
void UnscentedKalmanFilter<STATE_SIZE, TYPE>::predict(Command<TYPE>* const command)
{
    // Calculate the sigma points
    //
    // CHI = o.calculateSigmaPoints(XX, P);
    cv::Mat CHI = calculateSigmaPoints(state_, covariance_);

    // Pass the sigma points through g()
    //
    // Y = zeros(size(CHI));
    // for i = 1:size(CHI, 2);
    //     Y(:, i) = gFunction(CHI(:, i), gParams);
    // end
    cv::Mat Y = Math::zeros(CHI);
    for (unsigned int i = 0; i < CHI.cols; ++i)
    {
        cv::Mat T = process_.transformWithAB(CHI.col(i), command, dT_);
        T.copyTo(Y.col(i));
    }

    // [XX, S] = o.utTransform(XX, Y, CHI);
    cv::Mat C = Math::zeros(covariance_);
    unscentedTransform(state_, Y, CHI, state_, covariance_, C);

    // S = S + o.robot.getProfile().Q;
    covariance_ += process_.getNoiseMatrix();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
template<unsigned int STATE_SIZE, int TYPE>
void UnscentedKalmanFilter<STATE_SIZE, TYPE>::unscentedTransform(
        const cv::Mat XX, const cv::Mat Y, const cv::Mat CHI,
        cv::Mat& Ybar, cv::Mat& S, cv::Mat& C)
{
    // Calculate the predicted mean
    //
    // Ybar = zeros(size(Y, 1), 1);
    // for i = 1 : size(CHI, 2)
    //     Ybar = Ybar + o.WM(i) * Y(:, i);
    // end
    Ybar = Math::zeros(XX);

    for (unsigned int i = 0; i < Y.cols; ++i)
    {
        Ybar += WM_.at<BaseType>(i) * Y.col(i);
    }

    // Calculate the predicted covariance matrix
    //
    // S  = zeros(size(Y, 1), size(Y, 1));
    // C  = zeros(size(XX, 1), size(Y, 1));
    // for i = 1 : size(CHI, 2)
    //     S = S + o.WC(i) * (Y(:, i) - Ybar) * (Y(:, i) - Ybar)';
    //     C = C + o.WC(i) * (CHI(1:size(XX, 1), i) - XX) * (Y(:, i) - Ybar)';
    // end
    S.setTo(0);
    C.setTo(0);

    cv::Mat Ts = Math::zeros(S);

    for (unsigned int i = 0; i < Y.cols; ++i)
    {
        S += WC_.at<BaseType>(i) * (Y.col(i) - Ybar) * (Y.col(i) - Ybar).t();
        C += WC_.at<BaseType>(i) * (CHI.col(i) - XX) * (Y.col(i) - Ybar).t();
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
template<unsigned int STATE_SIZE, int TYPE>
void UnscentedKalmanFilter<STATE_SIZE, TYPE>::update(
        const vector<Measurement<STATE_SIZE, TYPE>*> measurements)
{
    for (auto measurement : measurements)
    {
        // Calculate the sigma points
        //
        // CHI = o.calculateSigmaPoints(XX, P);
        cv::Mat CHI = calculateSigmaPoints(state_, covariance_);

        Sensor<STATE_SIZE, TYPE>* sensor = measurement->getSensor();

        // Pass the sigma points through h()
        //
        // Y = zeros(size(CHI));
        // for i = 1:size(CHI, 2);
        //     Y(:, i) = hFunction(CHI(:, i));
        // end
        cv::Mat Y = Math::zeros(CHI);
        for (unsigned int i = 0; i < CHI.cols; ++i)
        {
            cv::Mat T = sensor->transformWithH(CHI.col(i));
            T.copyTo(Y.col(i));
        }

        // [Ybar, S, C] = o.utTransform(XX, Y, CHI);
        cv::Mat Ybar = Math::zeros(state_);
        cv::Mat S = Math::zeros(covariance_);
        cv::Mat C = Math::zeros(covariance_);
        unscentedTransform(state_, Y, CHI, Ybar, S, C);

        // S = S + sensor.getR();
        S += sensor->getNoiseMatrix();

        // S = o.checkUnderflow(S);
        checkDiagonalUnderflow(S);

        // z = sensor.measurement2state(measurement);
        cv::Mat z = sensor->transform2State(measurement);

        // K = C / S;
        cv::Mat K = C * S.inv();

        // XX = XX + K * (z - Ybar);
        state_ += K * (z - Ybar);

        // P = P - K * S * K';
        covariance_ -= K * S * K.t();
    }
}

} // namespace srs
