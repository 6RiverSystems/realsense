// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved

#include "../include/realsense_node_factory.h"
#include "../include/base_realsense_node.h"
#include "../include/t265_realsense_node.h"
#include <iostream>
#include <map>
#include <mutex>
#include <condition_variable>
#include <signal.h>
#include <thread>
#include <sys/time.h>
#include <regex>
#include <algorithm>
#include <std_msgs/String.h>

using namespace realsense2_camera;

#define REALSENSE_ROS_EMBEDDED_VERSION_STR (VAR_ARG_STRING(VERSION: REALSENSE_ROS_MAJOR_VERSION.REALSENSE_ROS_MINOR_VERSION.REALSENSE_ROS_PATCH_VERSION))
constexpr auto realsense_ros_camera_version = REALSENSE_ROS_EMBEDDED_VERSION_STR;

PLUGINLIB_EXPORT_CLASS(realsense2_camera::RealSenseNodeFactory, nodelet::Nodelet)

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
	_initialized = false;
}

void RealSenseNodeFactory::closeDevice()
{
	for (rs2::sensor sensor : _device.query_sensors())
	{
		sensor.stop();
		sensor.close();
	}
}

RealSenseNodeFactory::~RealSenseNodeFactory()
{
	closeDevice();
}

void RealSenseNodeFactory::getDevice(rs2::device_list list)
{
	if (!_device)
	{
		if (0 == list.size())
		{
			ROS_WARN("No RealSense devices were found!");
		}
		else
		{
			bool found = false;
			ROS_INFO_STREAM(" ");
			for (auto &&dev : list)
			{
				auto sn = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
				ROS_INFO_STREAM("Device with serial number " << sn << " was found." << std::endl);
				std::string pn = dev.get_info(RS2_CAMERA_INFO_PHYSICAL_PORT);
				std::string name = dev.get_info(RS2_CAMERA_INFO_NAME);
				ROS_INFO_STREAM("Device with physical ID " << pn << " was found.");
				std::string port_id;
				std::vector<std::string> results;
				ROS_INFO_STREAM("Device with name " << name << " was found.");
				std::regex self_regex;
				if (name == std::string("Intel RealSense T265"))
				{
					self_regex = std::regex(".*?bus_([0-9]+) port_([0-9]+).*", std::regex_constants::ECMAScript);
				}
				else // if(strcmp(name, "Intel RealSense D435") == 0)
				{
					self_regex = std::regex("[^ ]+/usb[0-9]+[0-9./-]*/([0-9.-]+):[^ ]*", std::regex_constants::ECMAScript);
				}
				std::smatch base_match;
				bool found_usb_desc = std::regex_match(pn, base_match, self_regex);
				if (found_usb_desc)
				{
					std::ssub_match base_sub_match = base_match[1];
					port_id = base_sub_match.str();
					for (unsigned int mi = 2; mi < base_match.size(); mi++)
					{
						std::ssub_match base_sub_match = base_match[mi];
						port_id += "-" + base_sub_match.str();
					}
					ROS_INFO_STREAM("Device with port number " << port_id << " was found.");
				}
				else
				{
					std::stringstream msg;
					msg << "Error extracting usb port from device with physical ID: " << pn << std::endl << "Please report on github issue at https://github.com/IntelRealSense/realsense-ros";
					if (_usb_port_id.empty())
					{
						ROS_WARN_STREAM(msg.str());
					}
					else
					{
						ROS_ERROR_STREAM(msg.str());
						ROS_ERROR_STREAM("Please use serial number instead of usb port.");
					}
				}
				bool found_device_type(true);
				if (!_device_type.empty())
				{
					std::regex device_type_regex(_device_type.c_str(), std::regex::icase);
					found_device_type = std::regex_search(name, base_match, device_type_regex);
				}

				if ((_serial_no.empty() || sn == _serial_no) && (_usb_port_id.empty() || port_id == _usb_port_id) && found_device_type)
				{
					_device = dev;
					_serial_no = sn;
					found = true;
					break;
				}
			}
			if (!found)
			{
				// T265 could be caught by another node.
				std::string msg("The requested device with ");
				bool add_and(false);
				if (!_serial_no.empty())
				{
					msg += "serial number " + _serial_no;
					add_and = true;
				}
				if (!_usb_port_id.empty())
				{
					if (add_and)
					{
						msg += " and ";
					}
					msg += "usb port id " + _usb_port_id;
					add_and = true;
				}
				if (!_device_type.empty())
				{
					if (add_and)
					{
						msg += " and ";
					}
					msg += "device name containing " + _device_type;
				}
				msg += " is NOT found. Will Try again.";
				ROS_ERROR_STREAM(msg);
			}
		}
	}

	bool remove_tm2_handle(_device && RS_T265_PID != std::stoi(_device.get_info(RS2_CAMERA_INFO_PRODUCT_ID), 0, 16));
	if (remove_tm2_handle)
	{
		_ctx.unload_tracking_module();
	}

	if (_device && _initial_reset)
	{
		_initial_reset = false;
		try
		{
			ROS_INFO("Resetting device...");
			_device.hardware_reset();
			_device = rs2::device();
		}
		catch (const std::exception &ex)
		{
			ROS_WARN_STREAM("An exception has been thrown: " << ex.what());
		}
	}
}

void RealSenseNodeFactory::change_device_callback(rs2::event_information &info)
{
	if (info.was_removed(_device))
	{
		ROS_ERROR("The device has been disconnected!");
		_realSenseNode.reset(nullptr);
		_device = rs2::device();
	}
	if (!_device)
	{
		rs2::device_list new_devices = info.get_new_devices();
		if (new_devices.size() > 0)
		{
			ROS_INFO("Checking new devices...");
			getDevice(new_devices);
			if (_device)
			{
				StartDevice();
			}
		}
	}
}

void RealSenseNodeFactory::oldOnInit()
{
	try
	{
#ifdef BPDEBUG
		std::cout << "Attach to Process: " << getpid() << std::endl;
		std::cout << "Press <ENTER> key to continue." << std::endl;
		std::cin.get();
#endif
		ros::NodeHandle nh = getNodeHandle();
		auto privateNh = getPrivateNodeHandle();
		privateNh.param("serial_no", _serial_no, std::string(""));
		privateNh.param("usb_port_id", _usb_port_id, std::string(""));
		privateNh.param("device_type", _device_type, std::string(""));

		std::string rosbag_filename("");
		privateNh.param("rosbag_filename", rosbag_filename, std::string(""));

		if (!rosbag_filename.empty())
		{
			{
				ROS_INFO_STREAM("publish topics from rosbag file: " << rosbag_filename.c_str());
				auto pipe = std::make_shared<rs2::pipeline>();
				rs2::config cfg;
				cfg.enable_device_from_file(rosbag_filename.c_str(), false);
				cfg.enable_all_streams();
				pipe->start(cfg); //File will be opened in read mode at this point
				_device = pipe->get_active_profile().get_device();
				_serial_no = _device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
				_realSenseNode = std::unique_ptr<BaseRealSenseNode>(new BaseRealSenseNode(nh, privateNh, _device, _serial_no));
			}
			if (_device)
			{
				StartDevice();
			}
		}
		else
		{
			privateNh.param("initial_reset", _initial_reset, false);

			_query_thread = std::thread([=]() {
				std::chrono::milliseconds timespan(6000);
				while (!_device)
				{
					// _ctx.init_tracking_module(); // Unavailable function.
					getDevice(_ctx.query_devices());
					if (_device)
					{
						std::function<void(rs2::event_information &)> change_device_callback_function = [this](rs2::event_information &info) { change_device_callback(info); };
						_ctx.set_devices_changed_callback(change_device_callback_function);
						StartDevice();
					}
					else
					{
						std::this_thread::sleep_for(timespan);
					}
				}
			});
		}
	}
	catch (const std::exception &ex)
	{
		ROS_ERROR_STREAM(" An exception has been thrown: " << ex.what());
		exit(1);
	}
	catch (...)
	{
		ROS_ERROR_STREAM(" Unknown exception has occurred!");
		exit(1);
	}
}

void RealSenseNodeFactory::StartDevice()
{
	ros::NodeHandle nh = getNodeHandle();
	ros::NodeHandle privateNh = getPrivateNodeHandle();
	// TODO
	std::string pid_str(_device.get_info(RS2_CAMERA_INFO_PRODUCT_ID));
	uint16_t pid = std::stoi(pid_str, 0, 16);
	switch (pid)
	{
	case SR300_PID:
	case SR300v2_PID:
	case RS400_PID:
	case RS405_PID:
	case RS410_PID:
	case RS460_PID:
	case RS415_PID:
	case RS420_PID:
	case RS420_MM_PID:
	case RS430_PID:
	case RS430_MM_PID:
	case RS430_MM_RGB_PID:
	case RS435_RGB_PID:
	case RS435i_RGB_PID:
	case RS_USB2_PID:
		_realSenseNode = std::unique_ptr<BaseRealSenseNode>(new BaseRealSenseNode(nh, privateNh, _device, _serial_no));
		break;
	case RS_T265_PID:
		_realSenseNode = std::unique_ptr<T265RealsenseNode>(new T265RealsenseNode(nh, privateNh, _device, _serial_no));
		break;
	default:
		ROS_FATAL_STREAM("Unsupported device!"
										 << " Product ID: 0x" << pid_str);
		ros::shutdown();
		exit(1);
	}
	assert(_realSenseNode);
	try
	{
		_realSenseNode->publishTopics();
	}
	catch (...)
	{
		resetAndShutdown();
	}
}

// Code backported for use by 6RS starts below

void RealSenseNodeFactory::onInit()
{
	auto privateNh = getPrivateNodeHandle();
	auto nodeHandle = getNodeHandle();
	bool wait_for_usb_resetter = true;
	std::string analytics_topic;
	privateNh.param("usb_port_id", _usb_port_id, std::string(""));
	privateNh.param("wait_for_usb_resetter", wait_for_usb_resetter, true);
	if (wait_for_usb_resetter)
	{
		ROS_INFO_STREAM(__FILE__ << " " << __LINE__ << "realsense_camera: device " << _usb_port_id << " wait_for_usb_resetter enabled!");
		ros::SubscriberStatusCallback connect_cb = boost::bind(&RealSenseNodeFactory::connectCb, this);
		ROS_INFO_STREAM(__FILE__ << " " << __LINE__ << "realsense_camera: device " << _usb_port_id << " Advertising reset request publisher!");
		_reset_request_publisher = nodeHandle.advertise<std_msgs::String>("reset_request", 10, connect_cb);
		ROS_INFO_STREAM(__FILE__ << " " << __LINE__ << "realsense_camera: device " << _usb_port_id << " Reset request publisher advertised!");

		_setupOneShotTimer = nodeHandle.createTimer(ros::Duration(30), boost::bind(&RealSenseNodeFactory::connectCb, this), true);
	}
	else
	{
		ROS_INFO_STREAM(__FILE__ << " " << __LINE__ << "realsense_camera: device " << _usb_port_id << " wait_for_usb_resetter disabled!");
		ROS_INFO_STREAM(__FILE__ << " " << __LINE__ << "realsense_camera: device " << _usb_port_id << " Advertising reset request publisher!");
		_reset_request_publisher = nodeHandle.advertise<std_msgs::String>("reset_request", 10);
		ROS_INFO_STREAM(__FILE__ << " " << __LINE__ << "realsense_camera: device " << _usb_port_id << " Reset request publisher advertised!");

		_setupOneShotTimer = nodeHandle.createTimer(ros::Duration(1), boost::bind(&RealSenseNodeFactory::connectCb, this), true);
	}
}

void RealSenseNodeFactory::tryGetLogSeverity(rs2_log_severity &severity) const
{
	static const char *severity_var_name = "LRS_LOG_LEVEL";
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

void RealSenseNodeFactory::resetAndShutdown()
{
	std::lock_guard<std::recursive_mutex> lock(_device_lock);
	if (_realSenseNode)
	{
		try
		{
			_realSenseNode->stopStreams();
			// this might be handled by close device on destructor...
		}
		catch (...)
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
		}
		catch (...)
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
	REQUEST_SHUTDOWN();
}

void RealSenseNodeFactory::notification_handler(const rs2::notification &n, size_t iteration)
{
	std::lock_guard<std::recursive_mutex> scopedLock(_device_lock);
	if (ros::isShuttingDown())
	{
		ROS_INFO_STREAM("realsense_camera: device " << _usb_port_id << " ROS is shutting down returning early");
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
		ROS_INFO_STREAM("realsense_camera: device " << _usb_port_id << " received a notification that does not require reset. The category was: " << rs2_notification_category_to_string(n.get_category()));
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
		ROS_INFO_STREAM("realsense_camera: device " << _usb_port_id << " unknown exception has occurred while shutting down streams. ignoring...");
	}

	ROS_INFO_STREAM("realsense_camera: Device " << _usb_port_id << " creating new processing thread");

	std::thread([this, n]() {
		try
		{
			RETURN_ON_SHUTDOWN();

			ROS_INFO_STREAM("realsense_camera: new procesing thread started executing");
			ROS_INFO_STREAM("realsense_camera: device " << _usb_port_id << " deallocating realsensenode");
			try
			{
				_realSenseNode.reset();
			}
			catch (...)
			{
				ROS_INFO_STREAM("realsense_camera: device " << _usb_port_id << " unknown exception has occurred while resetting Realsense node. Ignoring...");
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
				ROS_INFO_STREAM("realsense_camera: device " << _usb_port_id	<< " unknown exception has occurred while creating a new device. Ignoring...");
			}
			try
			{
				_ctx = rs2::context{};
			}
			catch (...)
			{
				ROS_INFO_STREAM("realsense_camera: device " << _usb_port_id << " unknown exception has occurred while creating a new context. Ignoring...");
			}
			ROS_INFO_STREAM("realsense_camera: device " << _usb_port_id << " sleeping for 1 second");
			boost::this_thread::sleep(boost::posix_time::seconds(1));
			bool found = false;
			while (!found)
			{
				RETURN_ON_SHUTDOWN();
				for (auto &&dev : _ctx.query_devices())
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
							ROS_INFO_STREAM("realsense_camera: device " << _usb_port_id	<< " unknown exception has occurred while resetting hardware. Ignoring...");
						}

						ROS_INFO_STREAM("realsense_camera: device " << _usb_port_id << " found... and was reset");
						found = true;
						break;
					}
				}
				if (!found)
				{
					ROS_INFO_STREAM("realsense_camera: device " << _usb_port_id	<< " not found... and not reset... sleeping for 2 seconds");
					boost::this_thread::sleep(boost::posix_time::seconds(2));
					std_msgs::String msg;
					msg.data = _sensor_name + _usb_port_id;
					_reset_request_publisher.publish(msg);
					// need to add analytics messages here
					srslib_framework::Event data;
					data.entity = _sensor_entity;
					data.value = _sensor_value;
					data.attribute = _sensor_name + _usb_port_id + ".USB_BUS_RESET_REQUEST_SENT";
					_reset_request_analytics_publisher.publish(data);
				}
			}

			RETURN_ON_SHUTDOWN();
			ROS_INFO_STREAM("realsense_camera: device " << _usb_port_id << " sleeping for 10 seconds");
			boost::this_thread::sleep(boost::posix_time::seconds(10));

			found = false;
			while (!found)
			{
				RETURN_ON_SHUTDOWN();

				for (auto &&dev : _ctx.query_devices())
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
							REQUEST_SHUTDOWN();
						}
						ROS_INFO_STREAM(
								"realsense_camera: device " << _usb_port_id << " found... and was added");
						found = true;
						break;
					}
				}
				if (!found)
				{
					ROS_INFO_STREAM("realsense_camera: device " << _usb_port_id	<< " not found... sleeping for 2 seconds");
					boost::this_thread::sleep(boost::posix_time::seconds(2));
					std_msgs::String msg;
					msg.data = _sensor_name + _usb_port_id;
					_reset_request_publisher.publish(msg);
					// need to add analytics messages here
					srslib_framework::Event data;
					data.entity = _sensor_entity;
					data.value = _sensor_value;
					data.attribute = _sensor_name + _usb_port_id + ".USB_BUS_RESET_REQUEST_SENT";
					_reset_request_analytics_publisher.publish(data);
				}
			}
		}
		catch (...)
		{
			ROS_ERROR_STREAM("realsense_camera: device " << _usb_port_id << " an unexpected exception was detected while attempting to reset and add device. Requesting shut down of ros.");
			REQUEST_SHUTDOWN();
		}
	}).detach();

	ROS_INFO_STREAM("realsense_camera: Device " << _usb_port_id << " exiting notification handler");
}

void RealSenseNodeFactory::connectCb()
{
	{
		std::lock_guard<std::mutex> lock(_configurationMutex);
		if (_initialized == true)
		{
			return;
		}
		_initialized = true;
	}
	try
	{
		setUpChuck();
	}
	catch (const std::exception &ex)
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

void RealSenseNodeFactory::setUpChuck()
{
	auto privateNh = getPrivateNodeHandle();
	auto nodeHandle = getNodeHandle();

	privateNh.param("usb_port_id", _usb_port_id, std::string(""));

	{
		std::lock_guard<std::recursive_mutex> scopedLock(_device_lock);

		// Initial population of the device list
		bool found = false;
		while (!found)
		{
			RETURN_ON_SHUTDOWN();
			for (auto &&dev : _ctx.query_devices())
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
				ROS_ERROR_STREAM("realsense_camera: no devices found for adding... " << _usb_port_id << " sleeping for 2 seconds and polling again");
				boost::this_thread::sleep(boost::posix_time::seconds(2));
				_ctx = rs2::context{};
				std_msgs::String msg;
				msg.data = _sensor_name + _usb_port_id;
				_reset_request_publisher.publish(msg);
				// need to add analytics messages here
				srslib_framework::Event data;
				data.entity = _sensor_entity;
				data.value = _sensor_value;
				data.attribute = _sensor_name + _usb_port_id + ".USB_BUS_RESET_REQUEST_SENT";
				_reset_request_analytics_publisher.publish(data);
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
	while ((pos = usb_path.find(delimiter)) != std::string::npos)
	{
		temp = usb_path.substr(0, pos);
		split.push_back(temp);
		usb_path.erase(0, pos + delimiter.length());
	}

	if (split.size() < 3)
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

bool RealSenseNodeFactory::deviceMatches(rs2::device &dev, std::string &usb_port)
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

	switch (pid)
	{
	case SR300_PID:
	case SR300v2_PID:
	case RS400_PID:
	case RS405_PID:
	case RS410_PID:
	case RS460_PID:
	case RS415_PID:
	case RS420_PID:
	case RS420_MM_PID:
	case RS430_PID:
	case RS430_MM_PID:
	case RS430_MM_RGB_PID:
	case RS435_RGB_PID:
	case RS435i_RGB_PID:
	case RS_USB2_PID:
		_realSenseNode = std::unique_ptr<BaseRealSenseNode>(new BaseRealSenseNode(nh, privateNh, _device, _serial_no));
		break;
	case RS_T265_PID:
		_realSenseNode = std::unique_ptr<T265RealsenseNode>(new T265RealsenseNode(nh, privateNh, _device, _serial_no));
	default:
		ROS_FATAL_STREAM("Unsupported device!" << " Product ID: 0x" << pid_str);
		REQUEST_SHUTDOWN();
	}

	ROS_INFO("REALSENSE: Adding device on port %s", _device.get_info(RS2_CAMERA_INFO_PHYSICAL_PORT));

	assert(_realSenseNode);
	size_t local_copy_of_iteration = _device_iteration;
	_handler = [this, local_copy_of_iteration](const rs2::notification &n) {
		ROS_ERROR_STREAM("realsense_camera: callback for device " << _usb_port_id << " received. Invoking notification handler for processing.");
		notification_handler(n, local_copy_of_iteration);
	};
	_realSenseNode->publishTopics(_handler);
}