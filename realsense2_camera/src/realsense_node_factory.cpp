// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved

#include "../include/realsense_node_factory.h"
#include "../include/sr300_node.h"
#include "../include/rs415_node.h"
#include "../include/rs435_node.h"
#include <iostream>
#include <algorithm>
#include <thread>


#include <unistd.h>
#include <sys/reboot.h>

using namespace realsense2_camera;

#define REALSENSE_ROS_EMBEDDED_VERSION_STR (VAR_ARG_STRING(VERSION: REALSENSE_ROS_MAJOR_VERSION.REALSENSE_ROS_MINOR_VERSION.REALSENSE_ROS_PATCH_VERSION))
constexpr auto realsense_ros_camera_version = REALSENSE_ROS_EMBEDDED_VERSION_STR;

PLUGINLIB_EXPORT_CLASS(realsense2_camera::RealSenseNodeFactory, nodelet::Nodelet)


std::recursive_mutex RealSenseNodeFactory::_subsystemCallbackLock;

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


void RealSenseNodeFactory::notification_handler(const rs2::notification &n) {
    std::thread([this](){
        ROS_ERROR_STREAM("notification: Executing reset for device " << usb_port_id);
        ROS_ERROR_STREAM("notification: Device " << usb_port_id << " sleeping for 1 second");
        boost::this_thread::sleep(boost::posix_time::seconds(1));
        ROS_ERROR_STREAM("notification: Device " << usb_port_id << " stopic topics");
        try {
            _realSenseNode->stopTopics();
        } catch(...)
        {
            ROS_ERROR_STREAM("notification: Unknown exception has occurred while shutting down streams. ignoring...");
        }
        ROS_ERROR_STREAM("notification: Device " << usb_port_id << " sleeping for 1 second");
        boost::this_thread::sleep(boost::posix_time::seconds(1));
        ROS_ERROR_STREAM("notification: Device " << usb_port_id << " resetting");
        _device.hardware_reset();
        ROS_ERROR_STREAM("notification: Device " << usb_port_id << " sleeping for 10 seconds");
        boost::this_thread::sleep(boost::posix_time::seconds(10));
        ROS_ERROR_STREAM("notification: Device " << usb_port_id << " deallocating realsensenode");
        _realSenseNode.reset();
        ROS_ERROR_STREAM("notification: Device " << usb_port_id << " sleeping for 1 second");
        boost::this_thread::sleep(boost::posix_time::seconds(1));
        ROS_ERROR_STREAM("notification: Device " << usb_port_id << " creating a new device");
        _device = rs2::device();
        bool found = false;
        while(!found){
            for (auto &&dev : _context.query_devices()) {
                if (deviceMatches(dev, usb_port_id)) {
                    addDevice(dev);
                    ROS_ERROR_STREAM("notification: Device " << usb_port_id << " found... and was added");
                    found = true;
                    break;
                }
            }
            if(!found) {
                ROS_ERROR_STREAM("notification: Device " << usb_port_id << " not found... and was added");
            }
        }
    }).detach();
}

void RealSenseNodeFactory::onInit()
{
    try
    {
        auto privateNh = getPrivateNodeHandle();

        privateNh.param("usb_port_id", usb_port_id, std::string(""));

        {
            /*_context.set_devices_changed_callback([&](rs2::event_information& info)
            {
                std::lock_guard<std::recursive_mutex> scopedLock(_subsystemCallbackLock);

                removeDevice(info);

                for (auto&& dev : info.get_new_devices())
                {
                    if (deviceMatches(dev, usb_port_id))
                    {
                        addDevice(dev);
                    }
                }
            });
             */

            // Initial population of the device list
            bool found = false;
            while(!found) {
                    for (auto &&dev : _context.query_devices()) {
                        if (deviceMatches(dev, usb_port_id)) {
                            addDevice(dev);
                            found = true;
                            ROS_ERROR_STREAM("notification: Device " << usb_port_id << " found... and was added");

                            break;
                        }
                    }
                    if(!found) {
                        ROS_ERROR_STREAM("notification: No devices found for adding... " << usb_port_id << " sleeping for 2 seconds and polling again");
                        boost::this_thread::sleep(boost::posix_time::seconds(2));
                        _context = rs2::context{};
                    }
                }

            //boost::this_thread::sleep(boost::posix_time::seconds(10)); // don't release the lock until we know we don't need to shut down


        }
    }
    catch(const std::exception& ex)
    {
        ROS_ERROR_STREAM("An exception has been thrown: " << ex.what());

        resetAndShutdown();
    }
    catch(...)
    {
        ROS_ERROR_STREAM("Unknown exception has occurred!");

        resetAndShutdown();
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

    if(split.size() < 3)
    {
        ROS_ERROR("The USB port path is incorrect, return a empty string");
        return std::string();
    }

    // port id is the thrid element from last
    size_t port_id_index = split.size() - 3;

    // replace all '.' with '-' for consistency
    std::replace(split[port_id_index].begin(), split[port_id_index].end(), '.', '-');
    return split[port_id_index];
}

bool RealSenseNodeFactory::deviceMatches(rs2::device& dev, std::string& usb_port)
{
    std::string devicePhysicalPort = dev.get_info(RS2_CAMERA_INFO_PHYSICAL_PORT);

    ROS_DEBUG("Port ID: %s", devicePhysicalPort.c_str());

    std::string device_usb_port = parseUsbPortId(devicePhysicalPort);

    if (usb_port == device_usb_port)
    {
        return true;
    }
    else
    {
        return false;
    }
}

void RealSenseNodeFactory::addDevice(rs2::device dev)
{
    std::string serial_number(dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));

    //std::lock_guard<std::recursive_mutex> lock(_subsystemCallbackLock);

    // See if we already have the correct device attached
    if (_device)
    {
        ROS_INFO("REALSENSE: Device already added %s", _device.get_info(RS2_CAMERA_INFO_PHYSICAL_PORT));

        return;
    }

    _device = dev;

    auto privateNh = getPrivateNodeHandle();
    auto nh = getNodeHandle();

    auto pid_str = _device.get_info(RS2_CAMERA_INFO_PRODUCT_ID);
    uint16_t pid;
    std::stringstream ss;
    ss << std::hex << pid_str;
    ss >> pid;

    switch(pid)
    {
    case SR300_PID:
        _realSenseNode = std::unique_ptr<SR300Node>(new SR300Node(nh, privateNh, _device, ""));
        break;
    case RS400_PID:
        _realSenseNode = std::unique_ptr<BaseD400Node>(new BaseD400Node(nh, privateNh, _device, ""));
        break;
    case RS405_PID:
        _realSenseNode = std::unique_ptr<BaseD400Node>(new BaseD400Node(nh, privateNh, _device, ""));
        break;
    case RS410_PID:
    case RS460_PID:
        _realSenseNode = std::unique_ptr<BaseD400Node>(new BaseD400Node(nh, privateNh, _device, ""));
        break;
    case RS415_PID:
        _realSenseNode = std::unique_ptr<RS415Node>(new RS415Node(nh, privateNh, _device, ""));
        break;
    case RS420_PID:
        _realSenseNode = std::unique_ptr<BaseD400Node>(new BaseD400Node(nh, privateNh, _device, ""));
        break;
    case RS420_MM_PID:
        _realSenseNode = std::unique_ptr<BaseD400Node>(new BaseD400Node(nh, privateNh, _device, ""));
        break;
    case RS430_PID:
        _realSenseNode = std::unique_ptr<BaseD400Node>(new BaseD400Node(nh, privateNh, _device, ""));
        break;
    case RS430_MM_PID:
        _realSenseNode = std::unique_ptr<BaseD400Node>(new BaseD400Node(nh, privateNh, _device, ""));
        break;
    case RS430_MM_RGB_PID:
        _realSenseNode = std::unique_ptr<BaseD400Node>(new BaseD400Node(nh, privateNh, _device, ""));
        break;
    case RS435_RGB_PID:
        _realSenseNode = std::unique_ptr<RS435Node>(new RS435Node(nh, privateNh, _device, ""));
        break;
    case RS_USB2_PID:
        _realSenseNode = std::unique_ptr<BaseD400Node>(new BaseD400Node(nh, privateNh, _device, ""));
        break;
    default:
        ROS_FATAL_STREAM("Unsupported device!" << " Product ID: 0x" << pid_str);
        ros::shutdown();
        exit(1);
    }

    ROS_INFO("REALSENSE: Adding device on port %s", _device.get_info(RS2_CAMERA_INFO_PHYSICAL_PORT));

    assert(_realSenseNode);
    handler =
    [this](const rs2::notification &n) {
        ROS_ERROR_STREAM("notification: Callback for device " << usb_port_id << " received. Launching a new thread and resetting");
        notification_handler(n);
    };

    _realSenseNode->publishTopics(handler);
    _realSenseNode->registerDynamicReconfigCb();
}

void RealSenseNodeFactory::removeDevice(const rs2::event_information& info)
{
    //std::lock_guard<std::recursive_mutex> lock(_subsystemCallbackLock);

    if (info.was_removed(_device))
    {
        ROS_INFO("REALSENSE: Removing device on port %s", _device.get_info(RS2_CAMERA_INFO_PHYSICAL_PORT));

        _realSenseNode.reset();
        _device = rs2::device();
    }
}

void RealSenseNodeFactory::resetAndShutdown()
{
    //std::lock_guard<std::recursive_mutex> lock(_subsystemCallbackLock);

    _device.hardware_reset();

    sleep(5);

    ros::shutdown();
    exit(1);
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
