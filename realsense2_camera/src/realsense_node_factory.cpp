// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved

#include "../include/realsense_node_factory.h"
#include "../include/sr300_node.h"
#include "../include/rs415_node.h"
#include "../include/rs435_node.h"
#include <thread>


using namespace realsense2_camera;

#define REALSENSE_ROS_EMBEDDED_VERSION_STR (VAR_ARG_STRING(VERSION: REALSENSE_ROS_MAJOR_VERSION.REALSENSE_ROS_MINOR_VERSION.REALSENSE_ROS_PATCH_VERSION))
constexpr auto realsense_ros_camera_version = REALSENSE_ROS_EMBEDDED_VERSION_STR;

PLUGINLIB_EXPORT_CLASS(realsense2_camera::RealSenseNodeFactory, nodelet::Nodelet)


RealSenseNodeFactory::RealSenseNodeFactory()
{
    ROS_INFO("RealSense ROS v%s", REALSENSE_ROS_VERSION_STR);
    ROS_INFO("Running with LibRealSense v%s", RS2_API_VERSION_STR);

    //signal(SIGINT, signalHandler);
    auto severity = rs2_log_severity::RS2_LOG_SEVERITY_WARN;
    tryGetLogSeverity(severity);
    if (rs2_log_severity::RS2_LOG_SEVERITY_DEBUG == severity)
        ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);

    rs2::log_to_console(severity);
}

void RealSenseNodeFactory::notification_handler(const rs2::notification &n, int iteration)
{
    std::lock_guard<std::recursive_mutex> scopedLock(_device_lock);
    if (ros::isShuttingDown())
    {
        ROS_ERROR_STREAM("ROS is shutting down returning early");
        _realSenseNode->stopStreams();
        return;
    }
    if (iteration != this->_device_iteration)
    {
        ROS_INFO_STREAM("realsense_camera: device " << _usb_port_id << " iterations don't match... ignoring duplicate notification");
        return;
    }
    if (n.get_category() != RS2_NOTIFICATION_CATEGORY_FRAMES_TIMEOUT && n.get_category() != RS2_NOTIFICATION_CATEGORY_UNKNOWN_ERROR)
    {
        ROS_ERROR_STREAM("realsense_camera: device " << _usb_port_id << " received a notification that does not require reset. The category was: "
                            << rs2_notification_category_to_string(n.get_category()));
        return;
    }
    this->_device_iteration++;
    ROS_ERROR_STREAM("realsense_camera: Device " << _usb_port_id << " received a notification that requires reset. The category was: " << rs2_notification_category_to_string(n.get_category()));
    ROS_INFO_STREAM("realsense_camera: Device " << _usb_port_id << " executing reset for device");
    ROS_INFO_STREAM("realsense_camera: Device " << _usb_port_id << " stoping topics");
    try
    {
        _realSenseNode->stopStreams();
    }
    catch (...)
    {
        ROS_ERROR_STREAM("realsense_camera: device " << _usb_port_id << " unknown exception has occurred while shutting down streams. ignoring...");
    }

    ROS_INFO_STREAM("realsense_camera: Device " << _usb_port_id << " creating new processing thread");

    std::thread([this, n]() {
        if (ros::isShuttingDown())
        {
            ROS_ERROR_STREAM("ROS is shutting down returning early");
            return;
        }
        ROS_INFO_STREAM("realsense_camera: new procesing thread started executing");
        ROS_INFO_STREAM("realsense_camera: device " << _usb_port_id << " deallocating realsensenode");
        try
        {
            _realSenseNode.reset();
        }
        catch (...)
        {
            ROS_ERROR_STREAM("realsense_camera: device " << _usb_port_id << " unknown exception has occurred while resetting Realsense node. Ignoring...");
        }
        ROS_INFO_STREAM("realsense_camera: device " << _usb_port_id << " sleeping for 1 second");
        boost::this_thread::sleep(boost::posix_time::seconds(1));
        ROS_INFO_STREAM("realsense_camera: device " << _usb_port_id << " creating a new device");
        try
        {
            _device = rs2::device();
        }
        catch (...)
        {
            ROS_ERROR_STREAM("realsense_camera: device " << _usb_port_id << " unknown exception has occurred while creating a new device. Ignoring...");
        }
        try
        {
            _context = rs2::context{};
        }
        catch (...)
        {
            ROS_ERROR_STREAM("realsense_camera: device " << _usb_port_id << " unknown exception has occurred while creating a new context. Ignoring...");
        }
        ROS_INFO_STREAM("realsense_camera: device " << _usb_port_id << " sleeping for 1 second");
        boost::this_thread::sleep(boost::posix_time::seconds(1));
        bool found = false;
        while (!found)
        {
            if (ros::isShuttingDown())
            {
                ROS_ERROR_STREAM("ROS is shutting down returning early");
                return;
            }
            for (auto &&dev : _context.query_devices())
            {
                if (deviceMatches(dev, _usb_port_id))
                {
                    ROS_INFO_STREAM("realsense_camera: device " << _usb_port_id << " resetting");
                    try
                    {
                        dev.hardware_reset();
                    }
                    catch (...)
                    {
                        ROS_ERROR_STREAM("realsense_camera: device " << _usb_port_id << " unknown exception has occurred while resetting hardware. Ignoring...");
                    }

                    ROS_INFO_STREAM("realsense_camera: device " << _usb_port_id << " found... and was reset");
                    found = true;
                    break;
                }
            }
            if (!found)
            {
                ROS_INFO_STREAM("realsense_camera: device " << _usb_port_id << " not found... and not reset... sleeping for 2 seconds");
                boost::this_thread::sleep(boost::posix_time::seconds(2));
            }
        }

        if (ros::isShuttingDown())
        {
            return;
        }
        ROS_INFO_STREAM("realsense_camera: device " << _usb_port_id << " sleeping for 10 seconds");
        boost::this_thread::sleep(boost::posix_time::seconds(10));

        found = false;
        while (!found)
        {
            if (ros::isShuttingDown())
            {
                ROS_ERROR_STREAM("ROS is shutting down returning early");
                return;
            }
            for (auto &&dev : _context.query_devices())
            {
                if (deviceMatches(dev, _usb_port_id))
                {
                    try
                    {
                        addDevice(dev);
                    }
                    catch (...)
                    {
                        ROS_ERROR_STREAM("realsense_camera: device " << _usb_port_id << " unknown exception has occurred while calling addDevice on new device. Exiting...");
                        exit(1);
                    }
                    ROS_INFO_STREAM("realsense_camera: device " << _usb_port_id << " found... and was added");
                    found = true;
                    break;
                }
            }
            if (!found)
            {
                ROS_INFO_STREAM("realsense_camera: device " << _usb_port_id << " not found... sleeping for 2 seconds");
                boost::this_thread::sleep(boost::posix_time::seconds(2));
            }
        }
    }).detach();

    ROS_INFO_STREAM("realsense_camera: Device " << _usb_port_id << " exiting notification handler");

}

void RealSenseNodeFactory::onInit()
{
    try
    {
        setUpResinChuck();
    }
    catch (const std::exception& ex)
    {
        ROS_ERROR_STREAM(__FILE__ << " " << __LINE__ << "realsense_camera: device " << _usb_port_id << " An exception has been thrown: " << ex.what());
        resetAndShutdown();
    }
    catch (...)
    {
        ROS_ERROR_STREAM(__FILE__ << " " << __LINE__ << "realsense_camera: device " << _usb_port_id << " Unknown exception has occurred!");
        resetAndShutdown();
    }
}

void RealSenseNodeFactory::setUpResinChuck()
{
    auto privateNh = getPrivateNodeHandle();

    privateNh.param("usb_port_id", _usb_port_id, std::string(""));

    {
        std::lock_guard<std::recursive_mutex> scopedLock(_device_lock);

        // Initial population of the device list
        bool found = false;
        while (!found)
        {
            if (ros::isShuttingDown())
            {
                ROS_ERROR_STREAM("ROS is shutting down returning early");
                return;
            }
            for (auto &&dev : _context.query_devices())
            {
                if (deviceMatches(dev, _usb_port_id))
                {
                    addDevice(dev);
                    found = true;
                    ROS_INFO_STREAM("realsense_camera: device " << _usb_port_id << " found... and was added");
                    break;
                }
            }
            if (!found)
            {
                ROS_ERROR_STREAM("realsense_camera: no devices found for adding... " << _usb_port_id
                                                                                         << " sleeping for 2 seconds and polling again");
                boost::this_thread::sleep(boost::posix_time::seconds(2));
                _context = rs2::context{};
            }
        }

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
    if (!dev)
    {
        return false;
    }
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

    std::lock_guard<std::recursive_mutex> lock(_device_lock);

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
    int local_copy_of_iteration = _device_iteration;
    _handler =
            [this, local_copy_of_iteration](const rs2::notification &n) {
                ROS_ERROR_STREAM("realsense_camera: callback for device " << _usb_port_id
                                                                      << " received. Invoking notification handler for processing.");
                notification_handler(n, local_copy_of_iteration);
            };
    _realSenseNode->publishTopics(_handler);
    _realSenseNode->registerDynamicReconfigCb();
}

void RealSenseNodeFactory::resetAndShutdown()
{
    std::lock_guard<std::recursive_mutex> lock(_device_lock);
    if (_realSenseNode)
    {
        try
        {
            _realSenseNode->stopStreams();
        } catch (...)
        {
            ROS_ERROR_STREAM(__FILE__ << " " << __LINE__ << " realsense_camera: unknown exception thrown when stopping streams: " << _usb_port_id);
        }
    }
    sleep(1);
    if (_device)
    {
        try
        {
            _device.hardware_reset();
        } catch (...)
        {
            ROS_ERROR_STREAM(__FILE__ << " " << __LINE__ << " realsense_camera: unknown exception thrown when doing hardware_reset: " << _usb_port_id);
        }
        try
        {
            _device = rs2::device();
        }
        catch (...)
        {
            ROS_ERROR_STREAM(__FILE__ << " " << __LINE__ << " realsense_camera: device " << _usb_port_id << " unknown exception has occurred while creating a new device. Ignoring...");
        }
    }
    ROS_INFO_STREAM(__FILE__ << " " << __LINE__ << " realsense_camera: device: " << _usb_port_id << " shutting down in 5s");
    sleep(5);
    ROS_INFO_STREAM(__FILE__ << " " << __LINE__ << " realsense_camera: device: " << _usb_port_id << " shutting down ROS and exiting.");
    if (ros::isShuttingDown())
    {
        return; // someone else is shutting us down
    }
    ros::shutdown();
    sleep(5);
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
