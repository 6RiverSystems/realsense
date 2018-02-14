// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved

#include "../include/realsense_node_factory.h"
#include "../include/sr300_node.h"
#include "../include/rs415_node.h"
#include "../include/rs435_node.h"
#include <iostream>
#include <map>

using namespace realsense_ros_camera;

#define REALSENSE_ROS_EMBEDDED_VERSION_STR (VAR_ARG_STRING(VERSION: REALSENSE_ROS_MAJOR_VERSION.REALSENSE_ROS_MINOR_VERSION.REALSENSE_ROS_PATCH_VERSION))
constexpr auto realsense_ros_camera_version = REALSENSE_ROS_EMBEDDED_VERSION_STR;

PLUGINLIB_EXPORT_CLASS(realsense_ros_camera::RealSenseNodeFactory, nodelet::Nodelet)

RealSenseNodeFactory::RealSenseNodeFactory()
{
    ROS_INFO("RealSense ROS v%s", REALSENSE_ROS_VERSION_STR);
    ROS_INFO("Running with LibRealSense v%s", RS2_API_VERSION_STR);

    signal(SIGINT, signalHandler);
    auto severity = rs2_log_severity::RS2_LOG_SEVERITY_WARN;
    tryGetLogSeverity(severity);
    if (rs2_log_severity::RS2_LOG_SEVERITY_DEBUG == severity)
        ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);

    rs2::log_to_console(severity);
}

void RealSenseNodeFactory::onInit()
{
    try{
        auto list = _ctx.query_devices();
        if (0 == list.size())
        {
            ROS_ERROR("No RealSense devices were found! Terminating RealSense Node...");
            ros::shutdown();
            exit(1);
        }

        // add ability to launch multiple cameras with usb port number
        ROS_INFO("%lu camera(s) detected in the usb bus", list.size());

        auto privateNh = getPrivateNodeHandle();
        auto nh = getNodeHandle();

        std::string serial_no("");
        std::string usb_port_id("");
        privateNh.param("serial_no", serial_no, std::string(""));
        privateNh.param("usb_port_id", usb_port_id, std::string(""));


        std::string serial_number;
        std::string port_id;
        bool found = false;
        for (auto&& dev : list)
        {
            serial_number = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
            port_id = parseUsbPortId(dev.get_info(RS2_CAMERA_INFO_PHYSICAL_PORT));
            if (usb_port_id.empty())
            {
                _device = dev;
                serial_no = serial_number;
                found = true;
                break;
            }
            else if (port_id == usb_port_id)
            {
                _device = dev;
                serial_no = serial_number;
                found = true;
                ROS_INFO_STREAM("Device connected with USB port: " << port_id << " (serial number: " << serial_number << ") was found.");
                break;
            }
        }

        if (!found)
        {
            ROS_FATAL_STREAM("The requested device at USB port: " << usb_port_id << " is NOT found!");
            ros::shutdown();
            exit(1);
        }

        // we found the device. Restart the device and wait for it to come up again

        ROS_INFO_STREAM("RESETING DEVICE: " << port_id << " with serial number: " << serial_no);
        _device.hardware_reset();
        ros::Duration(5).sleep();
        ROS_INFO_STREAM("Attempting to reacquire device: " << port_id << " with serial number: " << serial_no);

        list = _ctx.query_devices();
        if (0 == list.size())
        {
            ROS_ERROR("No RealSense devices were found! Terminating RealSense Node...");
            ros::shutdown();
            exit(1);
        }
        found = false;
        for (auto&& dev : list)
        {
            auto sn = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
            if (sn == serial_no )
            {
                _device = dev;
                serial_no = sn;
                found = true;
                break;
            }
        }

        if (!found)
        {
            ROS_FATAL_STREAM("The requested device at USB port: " << usb_port_id << " is NOT found!");
            ros::shutdown();
            exit(1);
        }


        _ctx.set_devices_changed_callback([this](rs2::event_information& info)
        {
            if (info.was_removed(_device))
            {
                ROS_FATAL("The device has been disconnected! Terminating RealSense Node...");
                ros::shutdown();
                exit(1);
            }
        });

        // TODO
        auto pid_str = _device.get_info(RS2_CAMERA_INFO_PRODUCT_ID);
        uint16_t pid;
        std::stringstream ss;
        ss << std::hex << pid_str;
        ss >> pid;
        switch(pid)
        {
        case SR300_PID:
            _realSenseNode = std::unique_ptr<SR300Node>(new SR300Node(nh, privateNh, _device, serial_no));
            break;
        case RS400_PID:
            _realSenseNode = std::unique_ptr<BaseD400Node>(new BaseD400Node(nh, privateNh, _device, serial_no));
            break;
        case RS405_PID:
            _realSenseNode = std::unique_ptr<BaseD400Node>(new BaseD400Node(nh, privateNh, _device, serial_no));
            break;
        case RS410_PID:
        case RS460_PID:
            _realSenseNode = std::unique_ptr<BaseD400Node>(new BaseD400Node(nh, privateNh, _device, serial_no));
            break;
        case RS415_PID:
            _realSenseNode = std::unique_ptr<RS415Node>(new RS415Node(nh, privateNh, _device, serial_no));
            break;
        case RS420_PID:
            _realSenseNode = std::unique_ptr<BaseD400Node>(new BaseD400Node(nh, privateNh, _device, serial_no));
            break;
        case RS420_MM_PID:
            _realSenseNode = std::unique_ptr<BaseD400Node>(new BaseD400Node(nh, privateNh, _device, serial_no));
            break;
        case RS430_PID:
            _realSenseNode = std::unique_ptr<BaseD400Node>(new BaseD400Node(nh, privateNh, _device, serial_no));
            break;
        case RS430_MM_PID:
            _realSenseNode = std::unique_ptr<BaseD400Node>(new BaseD400Node(nh, privateNh, _device, serial_no));
            break;
        case RS430_MM_RGB_PID:
            _realSenseNode = std::unique_ptr<BaseD400Node>(new BaseD400Node(nh, privateNh, _device, serial_no));
            break;
        case RS435_RGB_PID:
            _realSenseNode = std::unique_ptr<RS435Node>(new RS435Node(nh, privateNh, _device, serial_no));
            break;
        case RS_USB2_PID:
            _realSenseNode = std::unique_ptr<BaseD400Node>(new BaseD400Node(nh, privateNh, _device, serial_no));
            break;
        default:
            ROS_FATAL_STREAM("Unsupported device!" << "Product ID: " << pid_str);
            ros::shutdown();
            exit(1);
        }

        assert(_realSenseNode);
        _realSenseNode->publishTopics();
        _realSenseNode->registerDynamicReconfigCb();
    }
    catch(const std::exception& ex)
    {
        ROS_ERROR_STREAM("An exception has been thrown: " << ex.what());
        throw;
    }
    catch(...)
    {
        ROS_ERROR_STREAM("Unknown exception has occured!");
        throw;
    }
}

std::string RealSenseNodeFactory::parseUsbPortId(std::string usb_path) const
{
    // split the string
    std::string delimiter = "/";
    std::string temp;
    std::vector<std::string> split;
    size_t pos = 0;

    // store split substring in a vector
    while((pos = usb_path.find(delimiter)) != std::string::npos) {
        temp = usb_path.substr(0, pos);
        split.push_back(temp);
        usb_path.erase(0, pos + delimiter.length());
    }

    // replace 2-1.1 to 2-1-1 for consistency
    std::string port_id = split[6].replace(3, 1, "-");
    return port_id;
}

void RealSenseNodeFactory::tryGetLogSeverity(rs2_log_severity& severity) const
{
    static const char* severity_var_name = "LRS_LOG_LEVEL";
    auto content = getenv(severity_var_name);

    if (content)
    {
        std::string content_str(content);
        std::transform(content_str.begin(), content_str.end(), content_str.begin(), ::toupper);

        for (uint32_t i = 0; i < RS2_LOG_SEVERITY_COUNT; i++)
        {
            auto current = std::string(rs2_log_severity_to_string((rs2_log_severity)i));
            std::transform(current.begin(), current.end(), current.begin(), ::toupper);
            if (content_str == current)
            {
                severity = (rs2_log_severity)i;
                break;
            }
        }
    }
}
