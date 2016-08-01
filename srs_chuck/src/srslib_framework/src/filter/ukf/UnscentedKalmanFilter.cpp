#include <iostream>
#include <functional>

#include <srslib_framework/math/MatrixMath.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
template<unsigned int STATE_SIZE, unsigned int COMMAND_SIZE, int TYPE>
UnscentedKalmanFilter<STATE_SIZE, COMMAND_SIZE, TYPE>::UnscentedKalmanFilter(
    Process<STATE_SIZE, COMMAND_SIZE, TYPE>& process,
    BaseType alpha, BaseType kappa, BaseType beta) :
        BaseKalmanFilter<STATE_SIZE, COMMAND_SIZE, TYPE>(process),
        alpha_(alpha),
        beta_(beta),
        kappa_(kappa),
        lambda_(BaseType())
{
    initializeWeights();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Protected methods

////////////////////////////////////////////////////////////////////////////////////////////////////
template<unsigned int STATE_SIZE, unsigned int COMMAND_SIZE, int TYPE>
cv::Mat UnscentedKalmanFilter<STATE_SIZE, COMMAND_SIZE, TYPE>::addWeighted(
    const cv::Mat W, const cv::Mat X)
{
    cv::Mat R = MatrixMath::zeros(X);

    for (unsigned int i = 0; i < X.cols; ++i)
    {
        R += W.at<BaseType>(i) * X.col(i);
    }

    return R;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
template<unsigned int STATE_SIZE, unsigned int COMMAND_SIZE, int TYPE>
cv::Mat UnscentedKalmanFilter<STATE_SIZE, COMMAND_SIZE, TYPE>::calculateSigmaPoints(
    cv::Mat X, cv::Mat P)
{
    // A = chol(P)';
    cv::Mat A = MatrixMath::cholesky(P);

    // CHI = [zeros(size(X)) A -A];
    cv::Mat CHI;
    cv::Mat zM = MatrixMath::zeros(X);
    cv::Mat array[] = {zM, A, -A};
    cv::hconcat(array, 3, CHI);

    // CHI = o.c * CHI + repmat(X, 1, size(CHI, 2));
    cv::Mat repeatedX;
    cv::repeat(X, 1, CHI.cols, repeatedX);

    cv::scaleAdd(CHI, c_, repeatedX, CHI);

    return CHI;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
template<unsigned int STATE_SIZE, unsigned int COMMAND_SIZE, int TYPE>
void UnscentedKalmanFilter<STATE_SIZE, COMMAND_SIZE, TYPE>::initializeWeights()
{
    BaseType alpha2 = pow(alpha_, BaseType(2.0));

    // o.kappa = 3 - o.n;
    // Currently Kappa is specified in the constructor of the filter. There
    // are conflicting papers about how to correctly initialize this value
    // kappa_ = BaseType(3.0) - STATE_SIZE;

    // o.lambda = o.alpha ^ 2 * (o.n + o.kappa) - o.n;
    lambda_ = alpha2 * (STATE_SIZE + kappa_) - STATE_SIZE;

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

    BaseType zeroValue = lambda_ / (STATE_SIZE + lambda_);
    WM_.at<BaseType>(0) = zeroValue;
    WC_.at<BaseType>(0) = zeroValue + (BaseType(1.0) - alpha2 + beta_);

    // o.c = o.n + o.lambda;
    c_ = sqrt(STATE_SIZE + lambda_);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
template<unsigned int STATE_SIZE, unsigned int COMMAND_SIZE, int TYPE>
void UnscentedKalmanFilter<STATE_SIZE, COMMAND_SIZE, TYPE>::predict(BaseType dT,
    Command<COMMAND_SIZE, TYPE>* const command)
{
    // Calculate the sigma points
    //
    // CHI = o.calculateSigmaPoints(XX, P);
    cv::Mat CHI = calculateSigmaPoints(BaseKFType::x_, BaseKFType::P_);

    // Pass the sigma points through g()
    //
    // Y = zeros(size(CHI));
    // for i = 1:size(CHI, 2);
    //     Y(:, i) = gFunction(CHI(:, i), gParams);
    // end
    cv::Mat Y = MatrixMath::zeros(CHI);
    for (unsigned int i = 0; i < CHI.cols; ++i)
    {
        cv::Mat T = BaseKFType::process_.FB(CHI.col(i), command, dT);
        T.copyTo(Y.col(i));
    }

    // [XX, S] = o.utTransform(XX, Y, CHI);
    cv::Mat C = MatrixMath::zeros(BaseKFType::P_);
    unscentedTransform(BaseKFType::x_, Y, CHI, BaseKFType::x_, BaseKFType::P_, C);

    // S = S + o.robot.getProfile().Q;
    BaseKFType::P_ += BaseKFType::process_.getQ();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
template<unsigned int STATE_SIZE, unsigned int COMMAND_SIZE, int TYPE>
cv::Mat UnscentedKalmanFilter<STATE_SIZE, COMMAND_SIZE, TYPE>::residual(
    const cv::Mat A, const cv::Mat B)
{
    return A - B;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
template<unsigned int STATE_SIZE, unsigned int COMMAND_SIZE, int TYPE>
void UnscentedKalmanFilter<STATE_SIZE, COMMAND_SIZE, TYPE>::unscentedTransform(
        const cv::Mat X, const cv::Mat Y, const cv::Mat CHI,
        cv::Mat& Ybar, cv::Mat& S, cv::Mat& C)
{
    // Calculate the predicted mean
    //
    // Ybar = zeros(size(Y, 1), 1);
    // for i = 1 : size(CHI, 2)
    //     Ybar = Ybar + o.WM(i) * Y(:, i);
    // end

    // Perform the average of the sigma points. The function
    // isolated so that it's possible to define a specific
    // function that depends on the type of data in the sigma points
    Ybar = addWeighted(WM_, Y);

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

    for (unsigned int i = 0; i < Y.cols; ++i)
    {
        double weight = WC_.at<BaseType>(i);
        S += weight * residual(Y.col(i), Ybar) * residual(Y.col(i), Ybar).t();
        C += weight * residual(CHI.col(i), X) * residual(Y.col(i), Ybar).t();
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
template<unsigned int STATE_SIZE, unsigned int COMMAND_SIZE, int TYPE>
void UnscentedKalmanFilter<STATE_SIZE, COMMAND_SIZE, TYPE>::update()
{
    for (auto sensor : BaseKFType::sensors_)
    {
        if (sensor->newDataAvailable())
        {
            // Calculate the sigma points
            //
            // CHI = o.calculateSigmaPoints(XX, P);
            cv::Mat CHI = calculateSigmaPoints(BaseKFType::x_, BaseKFType::P_);

            // Pass the sigma points through h()
            //
            // Y = zeros(size(CHI));
            // for i = 1:size(CHI, 2);
            //     Y(:, i) = hFunction(CHI(:, i));
            // end
            cv::Mat Y = MatrixMath::zeros(CHI);
            for (unsigned int i = 0; i < CHI.cols; ++i)
            {
                cv::Mat T = sensor->H(CHI.col(i));
                T.copyTo(Y.col(i));
            }

            // [Ybar, S, C] = o.utTransform(XX, Y, CHI);
            cv::Mat Ybar = MatrixMath::zeros(BaseKFType::x_);
            cv::Mat S = MatrixMath::zeros(BaseKFType::P_);
            cv::Mat C = MatrixMath::zeros(BaseKFType::P_);
            unscentedTransform(BaseKFType::x_, Y, CHI, Ybar, S, C);

            // S = S + sensor.getR();
            S += sensor->getR();

            // S = o.checkUnderflow(S);
            MatrixMath::checkDiagonal(S, UNDERFLOW_THRESHOLD, UNDERFLOW_THRESHOLD);

            // z = sensor.measurement2state(measurement);
            cv::Mat z = sensor->getCurrentData();

            // K = C / S;
            cv::Mat K = C * S.inv();

            // XX = XX + K * (z - Ybar);
            BaseKFType::x_ += K * residual(z, Ybar);

            // P = P - K * S * K';
            BaseKFType::P_ -= K * S * K.t();
        }
    }

    MatrixMath::checkRange(BaseKFType::x_, UNDERFLOW_THRESHOLD, BaseType());
    MatrixMath::checkDiagonal(BaseKFType::P_, UNDERFLOW_THRESHOLD, UNDERFLOW_THRESHOLD);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods

} // namespace srs
