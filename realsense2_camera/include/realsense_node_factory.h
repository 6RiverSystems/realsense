// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2018 Intel Corporation. All Rights Reserved

#pragma once

#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include <librealsense2/hpp/rs_processing.hpp>
#include <librealsense2/rs_advanced_mode.hpp>
#include <cv_bridge/cv_bridge.h>
#include <constants.h>
#include <realsense2_camera/Extrinsics.h>
#include <realsense2_camera/IMUInfo.h>
#include <csignal>
#include <eigen3/Eigen/Geometry>
#include <fstream>
#include <thread>
#include <mutex>
#include "ReturnOnShutdown.h"
#include "srslib_framework/ros/channel/ChannelEvent.hpp"

namespace realsense2_camera
{
    const stream_index_pair COLOR{RS2_STREAM_COLOR, 0};
    const stream_index_pair DEPTH{RS2_STREAM_DEPTH, 0};
    const stream_index_pair INFRA1{RS2_STREAM_INFRARED, 1};
    const stream_index_pair INFRA2{RS2_STREAM_INFRARED, 2};
    const stream_index_pair FISHEYE{RS2_STREAM_FISHEYE, 0};
    const stream_index_pair FISHEYE1{RS2_STREAM_FISHEYE, 1};
    const stream_index_pair FISHEYE2{RS2_STREAM_FISHEYE, 2};
    const stream_index_pair GYRO{RS2_STREAM_GYRO, 0};
    const stream_index_pair ACCEL{RS2_STREAM_ACCEL, 0};
    const stream_index_pair POSE{RS2_STREAM_POSE, 0};
    

    const std::vector<stream_index_pair> IMAGE_STREAMS = {DEPTH, INFRA1, INFRA2,
                                                          COLOR,
                                                          FISHEYE,
                                                          FISHEYE1, FISHEYE2};

    const std::vector<stream_index_pair> HID_STREAMS = {GYRO, ACCEL, POSE};


    inline void signalHandler(int signum)
    {
        ROS_INFO_STREAM(strsignal(signum) << " Signal is received! Terminating RealSense Node...");
        ros::requestShutdown();
        ros::shutdown();
        sleep(5);
        // exit(signum) is not thread safe and should not be invoked in a multithreaded environment unless all the threads had been joined.
        // see: http://www.cplusplus.com/reference/cstdlib/exit/
        //exit(signum);
    }

    class InterfaceRealSenseNode
    {
    public:
        virtual void publishTopics(const std::function<void (const rs2::notification &n)> &handler = nullptr) = 0;
        virtual void stopStreams() = 0;
        virtual void registerDynamicReconfigCb(ros::NodeHandle& nh) = 0;
        virtual ~InterfaceRealSenseNode() = default;
    };

    class RealSenseNodeFactory : public nodelet::Nodelet
    {
    public:
        RealSenseNodeFactory();
        virtual ~RealSenseNodeFactory();
        std::recursive_mutex _device_lock;
        void notification_handler(const rs2::notification &n, size_t iteration);
        void connectCb();
        void setUpChuck();

    private:
        void closeDevice();
        void StartDevice();
        void change_device_callback(rs2::event_information& info);
        void getDevice(rs2::device_list list);
        virtual void onInit() override;
        void oldOnInit();
        void tryGetLogSeverity(rs2_log_severity& severity) const;
        void resetAndShutdown();

        void addDevice(rs2::device dev);
        std::string parseUsbPortId(std::string usb_path) const;
        bool deviceMatches(rs2::device& dev, std::string& usb_port);

        rs2::device _device;
        std::unique_ptr<InterfaceRealSenseNode> _realSenseNode;
        rs2::context _ctx;
        std::string _serial_no;
        std::string _usb_port_id;
        size_t _device_iteration = 0;

        std::string _device_type;
        bool _initial_reset;
        std::thread _query_thread;

        std::function<void(const rs2::notification &n)> _handler;

        ros::Publisher _reset_request_publisher;
        srs::ChannelEvent _reset_request_analytics_publisher;
        ros::Timer _setupOneShotTimer;
        std::mutex _configurationMutex;
        bool _initialized;
        std::string const _sensor_name {"RGBD_SENSOR_USB_"};
        std::string const _sensor_entity {"USB_BUS"};
        double const _sensor_value {1.0};

    };
}//end namespace
