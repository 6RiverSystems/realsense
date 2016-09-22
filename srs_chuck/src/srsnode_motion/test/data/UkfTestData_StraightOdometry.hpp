/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef UKFTESTDATA_STRAIGHTODOMETRY_HPP_
#define UKFTESTDATA_STRAIGHTODOMETRY_HPP_

namespace srs {

static const Odometry<> ODOMETRY_STEP_00 = Odometry<>::ZERO;
static const cv::Mat STATE_STEP_00 = (cv::Mat_<double>(UKF_STATE_SIZE, 1) <<
    1.000e+00,
    1.000e+00,
            0,
            0,
            0
);

static const Odometry<> ODOMETRY_STEP_01 = Odometry<>(Velocity<>(0, 0.5, 0));
static const cv::Mat STATE_STEP_01 = (cv::Mat_<double>(UKF_STATE_SIZE, 1) <<
    1.005e+00,
    1.000e+00,
            0,
    5.000e-01,
            0
);

static const Odometry<> ODOMETRY_STEP_02 = Odometry<>(Velocity<>(0, 1.0, 0));
static const cv::Mat STATE_STEP_02 = (cv::Mat_<double>(UKF_STATE_SIZE, 1) <<
    1.015e+00,
    1.000e+00,
            0,
    1.000e+00,
            0
);

static const Odometry<> ODOMETRY_STEP_03 = Odometry<>(Velocity<>(0, 1.5, 0));
static const cv::Mat STATE_STEP_03 = (cv::Mat_<double>(UKF_STATE_SIZE, 1) <<
    1.030e+00,
    1.000e+00,
            0,
    1.500e+00,
            0
);

static const Odometry<> ODOMETRY_STEP_04 = Odometry<>(Velocity<>(0, 2.00, 0));
static const cv::Mat STATE_STEP_04 = (cv::Mat_<double>(UKF_STATE_SIZE, 1) <<
    1.050e+00,
    1.000e+00,
            0,
    2.000e+00,
            0
);

static const Odometry<> ODOMETRY_STEP_05 = Odometry<>(Velocity<>(0, 2.0, 0));
static const cv::Mat STATE_STEP_05 = (cv::Mat_<double>(UKF_STATE_SIZE, 1) <<
    1.070e+00,
    1.000e+00,
            0,
    2.000e+00,
            0
);

static const Odometry<> ODOMETRY_STEP_06 = Odometry<>(Velocity<>(0, 2.0, 0));
static const cv::Mat STATE_STEP_06 = (cv::Mat_<double>(UKF_STATE_SIZE, 1) <<
    1.090e+00,
    1.000e+00,
            0,
    2.000e+00,
            0
);

static const Odometry<> ODOMETRY_STEP_07 = Odometry<>(Velocity<>(0, 2.0, 0));
static const cv::Mat STATE_STEP_07 = (cv::Mat_<double>(UKF_STATE_SIZE, 1) <<
    1.110e+00,
    1.000e+00,
            0,
    2.000e+00,
            0
);

static const Odometry<> ODOMETRY_STEP_08 = Odometry<>(Velocity<>(0, 2.0, 0));
static const cv::Mat STATE_STEP_08 = (cv::Mat_<double>(UKF_STATE_SIZE, 1) <<
    1.130e+00,
    1.000e+00,
            0,
    2.000e+00,
            0
);

static const Odometry<> ODOMETRY_STEP_09 = Odometry<>(Velocity<>(0, 2.0, 0));
static const cv::Mat STATE_STEP_09 = (cv::Mat_<double>(UKF_STATE_SIZE, 1) <<
    1.150e+00,
    1.000e+00,
            0,
    2.000e+00,
            0
);

static const Odometry<> ODOMETRY_STEP_10 = Odometry<>(Velocity<>(0, 1.5, 0));
static const cv::Mat STATE_STEP_10 = (cv::Mat_<double>(UKF_STATE_SIZE, 1) <<
    1.165e+00,
    1.000e+00,
            0,
    1.500e+00,
            0
);

static const Odometry<> ODOMETRY_STEP_11 = Odometry<>(Velocity<>(0, 1.0, 0));
static const cv::Mat STATE_STEP_11 = (cv::Mat_<double>(UKF_STATE_SIZE, 1) <<
    1.180e+00,
    1.000e+00,
            0,
    1.000e+00,
            0
);

static const Odometry<> ODOMETRY_STEP_12 = Odometry<>(Velocity<>(0, 0.5, 0));
static const cv::Mat STATE_STEP_12 = (cv::Mat_<double>(UKF_STATE_SIZE, 1) <<
    1.180e+00,
    1.000e+00,
            0,
    5.000e-01,
            0
);

static const Odometry<> ODOMETRY_STEP_13 = Odometry<>::ZERO;
static const cv::Mat STATE_STEP_13 = (cv::Mat_<double>(UKF_STATE_SIZE, 1) <<
    1.180e+00,
    1.000e+00,
            0,
            0,
            0
);

} // namespace srs

#endif // UKFTESTDATA_STRAIGHTODOMETRY_HPP_