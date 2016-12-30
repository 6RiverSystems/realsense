/*
 * HidIO.cpp
 *
 *  Created on: Apr 4, 2016
 *      Author: dan
 */

#include <srslib_framework/io/HidIO.hpp>
#include <srslib_framework/utils/Logging.hpp>
#include <srslib_framework/platform/Thread.hpp>
#include <thread>
#include <ros/ros.h>

namespace srs
{

HidIO::HidIO(const char* pszName, int32_t pid, int32_t vid) :
	initializedUsb_(false),
	claimedUsb_(false),
	name_(pszName),
	hotplugHandle_(0),
	sequence_(0),
	rxEndpointAddress_(-1),
	rxMaxPacketSize_(-1),
	txEndpointAddress_(-1),
	txMaxPacketSize_(-1),
	open_(false),
	deviceHandle_(nullptr),
	pid_(pid),
	vid_(vid),
	thread_(),
	readCallback_()
{
	int rc = libusb_init(nullptr);

	initializedUsb_ = rc == LIBUSB_SUCCESS;

	if (!initializedUsb_)
	{
		ROS_ERROR_NAMED(name_, "Failed to initialize libusb: %s", libusb_error_name(rc));
	}

	libusb_set_debug(nullptr, LIBUSB_LOG_LEVEL_WARNING);

	if(libusb_has_capability(LIBUSB_CAP_HAS_HOTPLUG) != 1)
	{
		ROS_ERROR_NAMED(name_, "Hotplug capabilities are not supported on this platform");
	}
}

HidIO::~HidIO()
{
	close();

	if (initializedUsb_)
	{
		libusb_exit(nullptr);
		initializedUsb_ = false;
	}
}

void HidIO::open(ConnectionCallbackFn connectionCallback,
	ReadCallbackFn readCallback)
{
	if(!isOpen())
	{
		open_ = true;

		connetionCallback_ = connectionCallback;

		readCallback_ = readCallback;

		registerHotPlug();
	}
}

bool HidIO::isOpen() const
{
	return open_ &&
		deviceHandle_ &&
		rxEndpointAddress_	!= -1 &&
		rxMaxPacketSize_	!= -1 &&
		txEndpointAddress_	!= -1 &&
		txMaxPacketSize_	!= -1 ;
}

void HidIO::close()
{
	open_ = false;

	if (connetionCallback_)
	{
		connetionCallback_(false);
		connetionCallback_ = {};
	}

	readCallback_ = {};

	if (deviceHandle_)
	{
		// Wait for all transfers to cancel
		for(auto transfer : transfers_)
		{
			libusb_cancel_transfer(transfer);
		}

		while(hasTransfers())
		{
			spinOnce();
		}

		releaseDevice();
	}
	else
	{
		transfers_.erase(transfers_.begin(), transfers_.end());
	}

	if (hotplugHandle_)
	{
		libusb_hotplug_deregister_callback(nullptr, hotplugHandle_);
		hotplugHandle_ = 0;
	}
}

void HidIO::spinOnce()
{
	timeval tv;
	memset(&tv, 0, sizeof(struct timeval));
	tv.tv_usec = 10000;
	libusb_handle_events_timeout_completed(nullptr, &tv, nullptr);
}

void HidIO::write(const std::vector<char>& buffer)
{
	if(isOpen())
	{
		if (buffer.size() > txMaxPacketSize_ - 1)
		{
			throw std::runtime_error("Invalid buffer size");
		}

		libusb_transfer* transfer  = libusb_alloc_transfer(0);

		// Allocate enough for the buffer
		uint8_t* data = static_cast<uint8_t*>(calloc(txMaxPacketSize_, sizeof(uint8_t)));

		// Write the data size
		data[0] = buffer.size();

		// Write the data
		memcpy(data + 1, &buffer[0], buffer.size());

//			ROS_ERROR_STREAM("WriteData (" <<  buffer.size() << "): " <<
//				ToHex(std::vector<char>(buffer.begin(), buffer.begin() + buffer.size())));

		libusb_fill_interrupt_transfer(transfer, deviceHandle_,
			txEndpointAddress_, data, txMaxPacketSize_, &HidIO::writeCompleted, this, 0);

		submitTransfer(transfer);
	}
	else
	{
		ROS_ERROR_NAMED(name_, "Attempt to write to hid IO which is not open");
	}
}

bool HidIO::checkSuccess(const char* action, int rc)
{
	bool success = (rc == LIBUSB_SUCCESS);

	if (!success)
	{
		if (rc == LIBUSB_ERROR_NO_DEVICE)
		{
			deviceHandle_ = nullptr;

			connetionCallback_(false);
		}

		ROS_ERROR_NAMED(name_, "%s: %s", action, libusb_error_name(rc));
	}

	return success;
}

void HidIO::read()
{
	if (isOpen())
	{
		libusb_transfer* transfer = libusb_alloc_transfer(0);

		uint8_t* data = static_cast<uint8_t*>(calloc(rxMaxPacketSize_, sizeof(uint8_t)));

		libusb_fill_interrupt_transfer(transfer, deviceHandle_,
			rxEndpointAddress_, data, rxMaxPacketSize_, &HidIO::readCompleted, this, 0);

		submitTransfer(transfer);
	}
}

void HidIO::registerHotPlug()
{
	int rc = libusb_hotplug_register_callback(nullptr,
		(libusb_hotplug_event)(LIBUSB_HOTPLUG_EVENT_DEVICE_ARRIVED|LIBUSB_HOTPLUG_EVENT_DEVICE_LEFT),
		LIBUSB_HOTPLUG_ENUMERATE,
		pid_,
		vid_,
		LIBUSB_HOTPLUG_MATCH_ANY,
		&HidIO::hotPlugCallback,
		static_cast<void*>(this),
		&hotplugHandle_);

	if (!checkSuccess("Failed to register for hot plug events", rc))
	{
		hotplugHandle_ = 0;
	}
}

int HidIO::hotPlugCallback(libusb_context* ctx, libusb_device* device,
	libusb_hotplug_event event, void* userData)
{
	HidIO* pThis = reinterpret_cast<HidIO*>(userData);

	pThis->hotPlugCallbackInternal( ctx, device, event);

	return 0;
}

int HidIO::hotPlugCallbackInternal(libusb_context* ctx, libusb_device* device,
	libusb_hotplug_event event)
{
	if (event == LIBUSB_HOTPLUG_EVENT_DEVICE_ARRIVED)
	{
		claimDevice(device);
	}

	return 0;
}

void HidIO::claimDevice(libusb_device* device)
{
	struct libusb_device_descriptor desrc = { 0 };

	int rc = libusb_open(device, &deviceHandle_);

	checkSuccess("Failed to open usb device", rc);

	if (rc == LIBUSB_SUCCESS)
	{
		// Detach the device from the kernel driver if it has claimed the device
		if (libusb_kernel_driver_active(deviceHandle_, 0) == 1)
		{
			rc = libusb_detach_kernel_driver(deviceHandle_, 0);

			checkSuccess("Failed to detach usb device from kernel", rc);
		}
	}

	if (rc == LIBUSB_SUCCESS)
	{
		rc = libusb_claim_interface(deviceHandle_, 0);

		checkSuccess("Failed to claim usb device", rc);

		claimedUsb_ = rc == LIBUSB_SUCCESS;
	}

	if (rc == LIBUSB_SUCCESS)
	{
		libusb_get_device_descriptor(device, &desrc);

		std::vector<char> nameTemp;
		nameTemp.reserve(128);

		const uint8_t descriptorSize = 128;

		char vendor[descriptorSize] = { '\0' };
		char product[descriptorSize] = { '\0' };

		libusb_get_string_descriptor_ascii(deviceHandle_, desrc.iManufacturer, (unsigned char*)vendor, descriptorSize);
		vendor_ = vendor;

		libusb_get_string_descriptor_ascii(deviceHandle_, desrc.iProduct,(unsigned char*)product, descriptorSize);
		product_ = product;
	}

	if (rc == LIBUSB_SUCCESS)
	{
		rc = lookupEndpoints();
	}

	if (rc == LIBUSB_SUCCESS)
	{
		releaseDevice();
	}
}

void HidIO::releaseDevice()
{
	if (deviceHandle_)
	{
		ROS_INFO("Hid releasing device: %p", deviceHandle_);

		if (claimedUsb_)
		{
			int rc = libusb_release_interface(deviceHandle_, 0);

			checkSuccess("Failed to release usb device", rc);
		}

		libusb_close(deviceHandle_);
		deviceHandle_ = nullptr;
	}

	product_ = "";
	vendor_ = "";
}

int HidIO::lookupEndpoints()
{
	rxEndpointAddress_	= -1;
	rxMaxPacketSize_	= -1;
	txEndpointAddress_	= -1;
	txMaxPacketSize_	= -1;

	libusb_device* device = libusb_get_device(deviceHandle_);

	libusb_config_descriptor* config = nullptr;
	int rc = libusb_get_active_config_descriptor(device, &config);

	if (rc == LIBUSB_SUCCESS)
	{
		const libusb_interface *interface = &config->interface[0];

		int numEndpoints = interface->altsetting[0].bNumEndpoints;

		for(int i = 0; i < numEndpoints; i++)
		{
			const libusb_endpoint_descriptor* ep = &interface->altsetting[0].endpoint[i];

			if((ep->bmAttributes & LIBUSB_TRANSFER_TYPE_MASK) == LIBUSB_TRANSFER_TYPE_INTERRUPT)
			{
				if((ep->bEndpointAddress & LIBUSB_ENDPOINT_DIR_MASK) == LIBUSB_ENDPOINT_IN)
				{
					rxEndpointAddress_	= ep->bEndpointAddress;
					rxMaxPacketSize_	= ep->wMaxPacketSize;
				}
				else if((ep->bEndpointAddress & LIBUSB_ENDPOINT_DIR_MASK) == LIBUSB_ENDPOINT_OUT)
				{
					txEndpointAddress_	= ep->bEndpointAddress;
					txMaxPacketSize_	= ep->wMaxPacketSize;
				}
			}
		}

		libusb_free_config_descriptor(config);
	}
	else
	{
		ROS_ERROR_NAMED(name_, "Failed to get active config descriptor: %s", libusb_error_name(rc));
	}

	if (rxEndpointAddress_	== -1	||
		rxMaxPacketSize_	== -1	||
		txEndpointAddress_	== -1	||
		txMaxPacketSize_	== -1)
	{
		ROS_ERROR_NAMED(name_, "Failed to get endpoint addresses and/or size");

		rc = LIBUSB_ERROR_NOT_FOUND;
	}

	if (rc == LIBUSB_SUCCESS)
	{
		ROS_INFO_NAMED(name_, "USB device connected: pid=0x%x, vid=0x%x, vendor=%s, product=%s rxAddr=0x%x rxSize=%d txAddr=0x%x txSize=%d",
			pid_, vid_, vendor_.c_str(), product_.c_str(),
			rxEndpointAddress_, rxMaxPacketSize_, txEndpointAddress_, txMaxPacketSize_);

		connetionCallback_(true);

		read();
	}

	return rc;
}

void HidIO::readCompleted(libusb_transfer* transfer)
{
	HidIO* pThis = reinterpret_cast<HidIO*>(transfer->user_data);

	pThis->readCompletedInternal(transfer);
}

bool HidIO::hasTransfers()
{
	return transfers_.size() > 0;
}

void HidIO::submitTransfer(libusb_transfer* transfer)
{
	transfer->flags |= LIBUSB_TRANSFER_FREE_BUFFER;
	transfer->flags |= LIBUSB_TRANSFER_FREE_TRANSFER;
	transfer->flags |= LIBUSB_TRANSFER_SHORT_NOT_OK;

	int rc = libusb_submit_transfer(transfer);

	if (checkSuccess("Hid submit transfer failed", rc))
	{
		transfers_.insert(transfer);
	}
}

bool HidIO::cleanupTransfer(libusb_transfer* transfer)
{
	bool success = true;

	if (transfer->status == LIBUSB_TRANSFER_NO_DEVICE)
	{
		deviceHandle_ = nullptr;
		connetionCallback_(false);

		success = false;
	}

	transfers_.erase(transfer);

	return success;
}

void HidIO::readCompletedInternal(libusb_transfer* transfer)
{
	if (transfer->status == LIBUSB_TRANSFER_COMPLETED)
	{
		uint8_t size = transfer->buffer[0];
		uint8_t counter = transfer->buffer[254];

		std::vector<char> message(transfer->buffer + 1, transfer->buffer + 1 + size);

		if(readCallback_)
		{
			if (sequence_ != counter)
			{
				ROS_ERROR_STREAM_NAMED(name_, "ReadData (size=" << (int)size << ", count=" << (int)counter << ", expected=" <<
					(int)sequence_ << " ): " <<
					ToHex(std::vector<char>(message.begin(), message.end())));
			}

			readCallback_(std::move(message));

			sequence_ = (counter + 1) % 256;
		}
		else
		{
			ROS_ERROR_NAMED(name_, "Hid IO read but no callback specified!\n");
		}
	}
	else if (transfer->status != LIBUSB_TRANSFER_STALL)
	{
		ROS_ERROR_NAMED(name_, "Hid read failed: %s", libusb_error_name(transfer->status));
	}

	if (cleanupTransfer(transfer))
	{
		read();
	}
}

void HidIO::writeCompleted(libusb_transfer* transfer)
{
	HidIO* pThis = reinterpret_cast<HidIO*>(transfer->user_data);

	pThis->writeCompletedInternal(transfer);
}

void HidIO::writeCompletedInternal(libusb_transfer* transfer)
{
	if (transfer->status != LIBUSB_TRANSFER_COMPLETED &&
		transfer->status != LIBUSB_TRANSFER_STALL)
	{
		ROS_ERROR_NAMED(name_, "Hid write failed: %s", libusb_error_name(transfer->status));
	}

	cleanupTransfer(transfer);
}

}
