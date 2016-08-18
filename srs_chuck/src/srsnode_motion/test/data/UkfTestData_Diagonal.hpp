/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef UKFTESTDATA_DIAGONAL_HPP_
#define UKFTESTDATA_DIAGONAL_HPP_

namespace srs {

static const CmdVelocity<> COMMAND_STEP_00 = CmdVelocity<>(Velocity<>::ZERO);
static const Odometry<> ODOMETRY_STEP_00 = Odometry<>::ZERO;
static const cv::Mat STATE_STEP_00 = (cv::Mat_<double>(UKF_STATE_SIZE, 1) <<
              0,
              0,
     1.5708e+00,
              0,
              0
);

static const CmdVelocity<> COMMAND_STEP_01 = CmdVelocity<>(Velocity<>(0.5, 0));
static const Odometry<> ODOMETRY_STEP_01 = Odometry<>(Velocity<>(5, 0.5, 0));
static const cv::Mat STATE_STEP_01 = (cv::Mat_<double>(UKF_STATE_SIZE, 1) <<
              0,
              0,
     1.5708e+00,
     5.0000e-01,
              0
);

static const CmdVelocity<> COMMAND_STEP_02 = CmdVelocity<>(Velocity<>(1.0, 0));
static const Odometry<> ODOMETRY_STEP_02 = Odometry<>(Velocity<>(10, 1.0, 0));
static const cv::Mat STATE_STEP_02 = (cv::Mat_<double>(UKF_STATE_SIZE, 1) <<
              0,
     2.4994e-01,
     1.5708e+00,
     1.0000e+00,
              0
);

static const CmdVelocity<> COMMAND_STEP_03 = CmdVelocity<>(Velocity<>(1.5, 0));
static const Odometry<> ODOMETRY_STEP_03 = Odometry<>(Velocity<>(15, 1.5, 0));
static const cv::Mat STATE_STEP_03 = (cv::Mat_<double>(UKF_STATE_SIZE, 1) <<
              0,
     7.4976e-01,
     1.5708e+00,
     1.5000e+00,
              0
);

static const CmdVelocity<> COMMAND_STEP_04 = CmdVelocity<>(Velocity<>(2.0, 0));
static const Odometry<> ODOMETRY_STEP_04 = Odometry<>(Velocity<>(20, 2.00, 0));
static const cv::Mat STATE_STEP_04 = (cv::Mat_<double>(UKF_STATE_SIZE, 1) <<
              0,
     1.4994e+00,
     1.5708e+00,
     2.0000e+00,
              0
);

static const CmdVelocity<> COMMAND_STEP_05 = CmdVelocity<>(Velocity<>(2.0, 0));
static const Odometry<> ODOMETRY_STEP_05 = Odometry<>(Velocity<>(25, 2.0, 0));
static const cv::Mat STATE_STEP_05 = (cv::Mat_<double>(UKF_STATE_SIZE, 1) <<
              0,
     2.4989e+00,
     1.5708e+00,
     2.0000e+00,
              0
);

static const CmdVelocity<> COMMAND_STEP_06 = CmdVelocity<>(Velocity<>(2.0, 0));
static const Odometry<> ODOMETRY_STEP_06 = Odometry<>(Velocity<>(30, 2.0, 0));
static const cv::Mat STATE_STEP_06 = (cv::Mat_<double>(UKF_STATE_SIZE, 1) <<
              0,
     3.4982e+00,
     1.5708e+00,
     2.0000e+00,
              0
);

static const CmdVelocity<> COMMAND_STEP_07 = CmdVelocity<>(Velocity<>(2.0, 0));
static const Odometry<> ODOMETRY_STEP_07 = Odometry<>(Velocity<>(35, 2.0, 0));
static const cv::Mat STATE_STEP_07 = (cv::Mat_<double>(UKF_STATE_SIZE, 1) <<
              0,
     4.4975e+00,
     1.5708e+00,
     2.0000e+00,
              0
);

static const CmdVelocity<> COMMAND_STEP_08 = CmdVelocity<>(Velocity<>(2.0, 0));
static const Odometry<> ODOMETRY_STEP_08 = Odometry<>(Velocity<>(40, 2.0, 0));
static const cv::Mat STATE_STEP_08 = (cv::Mat_<double>(UKF_STATE_SIZE, 1) <<
              0,
     5.4966e+00,
     1.5708e+00,
     2.0000e+00,
              0
);

static const CmdVelocity<> COMMAND_STEP_09 = CmdVelocity<>(Velocity<>(2.0, 0));
static const Odometry<> ODOMETRY_STEP_09 = Odometry<>(Velocity<>(45, 2.0, 0));
static const cv::Mat STATE_STEP_09 = (cv::Mat_<double>(UKF_STATE_SIZE, 1) <<
              0,
     6.4957e+00,
     1.5708e+00,
     2.0000e+00,
              0
);

static const CmdVelocity<> COMMAND_STEP_10 = CmdVelocity<>(Velocity<>(1.5, 0));
static const Odometry<> ODOMETRY_STEP_10 = Odometry<>(Velocity<>(50, 1.5, 0));
static const cv::Mat STATE_STEP_10 = (cv::Mat_<double>(UKF_STATE_SIZE, 1) <<
              0,
     7.4946e+00,
     1.5708e+00,
     1.5000e+00,
              0
);

static const CmdVelocity<> COMMAND_STEP_11 = CmdVelocity<>(Velocity<>(1.0, 0));
static const Odometry<> ODOMETRY_STEP_11 = Odometry<>(Velocity<>(55, 1.0, 0));
static const cv::Mat STATE_STEP_11 = (cv::Mat_<double>(UKF_STATE_SIZE, 1) <<
              0,
     8.2438e+00,
     1.5708e+00,
     1.0000e+00,
              0
);

static const CmdVelocity<> COMMAND_STEP_12 = CmdVelocity<>(Velocity<>(0.5, 0));
static const Odometry<> ODOMETRY_STEP_12 = Odometry<>(Velocity<>(60, 0.5, 0));
static const cv::Mat STATE_STEP_12 = (cv::Mat_<double>(UKF_STATE_SIZE, 1) <<
              0,
     8.7432e+00,
     1.5708e+00,
     5.0000e-01,
              0
);

static const CmdVelocity<> COMMAND_STEP_13 = CmdVelocity<>(Velocity<>(0, 0));
static const Odometry<> ODOMETRY_STEP_13 = Odometry<>(Velocity<>(65, 0, 0));
static const cv::Mat STATE_STEP_13 = (cv::Mat_<double>(UKF_STATE_SIZE, 1) <<
              0,
     8.9928e+00,
     1.5708e+00,
              0,
              0
);

} // namespace srs

#endif // UKFTESTDATA_DIAGONAL_HPP_
