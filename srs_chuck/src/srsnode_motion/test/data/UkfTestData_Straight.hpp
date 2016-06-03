/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef UKFTESTDATA_STRAIGHT_HPP_
#define UKFTESTDATA_STRAIGHT_HPP_

namespace srs {

const static Odometry<> ODOMETRY_STEP_00 = Odometry<>(0, 0, 0);
const static cv::Mat STATE_STEP_00 = (cv::Mat_<double>(UKF_STATE_SIZE, 1) <<
              0,
              0,
     1.5708e+00,
              0,
              0
);

const static Odometry<> ODOMETRY_STEP_01 = Odometry<>(0, 0.5, 0);
const static cv::Mat STATE_STEP_01 = (cv::Mat_<double>(UKF_STATE_SIZE, 1) <<
              0,
              0,
     1.5708e+00,
     5.0000e-01,
              0
);

const static Odometry<> ODOMETRY_STEP_02 = Odometry<>(0, 1.0, 0);
const static cv::Mat STATE_STEP_02 = (cv::Mat_<double>(UKF_STATE_SIZE, 1) <<
              0,
     2.4994e-01,
     1.5708e+00,
     1.0000e+00,
              0
);

const static Odometry<> ODOMETRY_STEP_03 = Odometry<>(0, 1.5, 0);
const static cv::Mat STATE_STEP_03 = (cv::Mat_<double>(UKF_STATE_SIZE, 1) <<
              0,
     7.4976e-01,
     1.5708e+00,
     1.5000e+00,
              0
);

const static Odometry<> ODOMETRY_STEP_04 = Odometry<>(0, 2.00, 0);
const static cv::Mat STATE_STEP_04 = (cv::Mat_<double>(UKF_STATE_SIZE, 1) <<
              0,
     1.4994e+00,
     1.5708e+00,
     2.0000e+00,
              0
);

const static Odometry<> ODOMETRY_STEP_05 = Odometry<>(0, 2.0, 0);
const static cv::Mat STATE_STEP_05 = (cv::Mat_<double>(UKF_STATE_SIZE, 1) <<
              0,
     2.4989e+00,
     1.5708e+00,
     2.0000e+00,
              0
);

const static Odometry<> ODOMETRY_STEP_06 = Odometry<>(0, 2.0, 0);
const static cv::Mat STATE_STEP_06 = (cv::Mat_<double>(UKF_STATE_SIZE, 1) <<
              0,
     3.4982e+00,
     1.5708e+00,
     2.0000e+00,
              0
);

const static Odometry<> ODOMETRY_STEP_07 = Odometry<>(0, 2.0, 0);
const static cv::Mat STATE_STEP_07 = (cv::Mat_<double>(UKF_STATE_SIZE, 1) <<
              0,
     4.4975e+00,
     1.5708e+00,
     2.0000e+00,
              0
);

const static Odometry<> ODOMETRY_STEP_08 = Odometry<>(0, 2.0, 0);
const static cv::Mat STATE_STEP_08 = (cv::Mat_<double>(UKF_STATE_SIZE, 1) <<
              0,
     5.4966e+00,
     1.5708e+00,
     2.0000e+00,
              0
);

const static Odometry<> ODOMETRY_STEP_09 = Odometry<>(0, 2.0, 0);
const static cv::Mat STATE_STEP_09 = (cv::Mat_<double>(UKF_STATE_SIZE, 1) <<
              0,
     6.4957e+00,
     1.5708e+00,
     2.0000e+00,
              0
);

const static Odometry<> ODOMETRY_STEP_10 = Odometry<>(0, 1.5, 0);
const static cv::Mat STATE_STEP_10 = (cv::Mat_<double>(UKF_STATE_SIZE, 1) <<
              0,
     7.4946e+00,
     1.5708e+00,
     1.5000e+00,
              0
);

const static Odometry<> ODOMETRY_STEP_11 = Odometry<>(0, 1.0, 0);
const static cv::Mat STATE_STEP_11 = (cv::Mat_<double>(UKF_STATE_SIZE, 1) <<
              0,
     8.2438e+00,
     1.5708e+00,
     1.0000e+00,
              0
);

const static Odometry<> ODOMETRY_STEP_12 = Odometry<>(0, 0.5, 0);
const static cv::Mat STATE_STEP_12 = (cv::Mat_<double>(UKF_STATE_SIZE, 1) <<
              0,
     8.7432e+00,
     1.5708e+00,
     5.0000e-01,
              0
);

const static Odometry<> ODOMETRY_STEP_13 = Odometry<>(0, 0, 0);
const static cv::Mat STATE_STEP_13 = (cv::Mat_<double>(UKF_STATE_SIZE, 1) <<
              0,
     8.9928e+00,
     1.5708e+00,
              0,
              0
);

} // namespace srs

#endif // UKFTESTDATA_STRAIGHT_HPP_
