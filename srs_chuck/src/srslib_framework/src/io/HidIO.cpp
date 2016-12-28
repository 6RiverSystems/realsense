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

struct Error : std::exception
{
    char text[1000];

    Error(char const* fmt, ...) __attribute__((format(printf,2,3))) {
        va_list ap;
        va_start(ap, fmt);
        vsnprintf(text, sizeof text, fmt, ap);
        va_end(ap);
    }

    char const* what() const throw() { return text; }
};

namespace srs
{

HidIO::HidIO(const char* pszName, int32_t pid, int32_t vid) :
	runEventThread_(true),
	name_(pszName),
	hotplugHandle_(0),
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
	try
	{
		int rc = libusb_init(nullptr);

		CheckSuccess(rc, "Failed to initialize libusb");

		libusb_set_debug(nullptr, LIBUSB_LOG_LEVEL_WARNING);

		if(libusb_has_capability(LIBUSB_CAP_HAS_HOTPLUG) != 1)
		{
			throw Error("Hotplug capabilities are not supported on this platform");
		}
	}
	catch(std::runtime_error& e)
	{
		ROS_ERROR("Hid Error: %s", e.what());
	}
}

HidIO::~HidIO()
{
	Close();

	libusb_exit(nullptr);
}

void HidIO::Open(ConnectionCallbackFn connectionCallback,
	ReadCallbackFn readCallback)
{
	if(!IsOpen())
	{
		try
		{
			open_ = true;

			connetionCallback_ = connectionCallback;

			readCallback_ = readCallback;

			RegisterHotPlug();
		}
		catch(std::runtime_error& e)
		{
			ROS_ERROR("Hid Error: %s", e.what());
		}
	}
}

bool HidIO::IsOpen() const
{
	return open_ &&
		deviceHandle_ &&
		rxEndpointAddress_	!= -1 &&
		rxMaxPacketSize_	!= -1 &&
		txEndpointAddress_	!= -1 &&
		txMaxPacketSize_	!= -1 ;
}

void HidIO::Close()
{
	try
	{
		open_ = false;

		if (connetionCallback_)
		{
			connetionCallback_(false);
			connetionCallback_ = {};
		}

		readCallback_ = {};

		bool hasTransfers = false;

		// Wait for all transfers to cancel
		for(auto transfer : transfers_)
		{
			libusb_cancel_transfer(transfer);
		}

		while(HasTransfers())
		{
			spinOnce();
		}

		ReleaseDevice();

		if (hotplugHandle_)
		{
			libusb_hotplug_deregister_callback(nullptr, hotplugHandle_);
			hotplugHandle_ = 0;
		}
	}
	catch(std::runtime_error& e)
	{
		ROS_ERROR("Hid Error: %s", e.what());
	}
	catch(...)
	{
		ROS_ERROR("Hid unknown error");
	}
}

void HidIO::spinOnce()
{
	timeval tv;
	memset(&tv, 0, sizeof(struct timeval));
	libusb_handle_events_timeout_completed(nullptr, &tv, nullptr);
}

void HidIO::Write(const std::vector<char>& buffer)
{
	try
	{
		if(IsOpen())
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
				txEndpointAddress_, data, txMaxPacketSize_, &HidIO::WriteCompleted, this, 0);

			SubmitTransfer(transfer);
		}
		else
		{
			throw std::runtime_error("Attempt to write to hid IO which is not open");
		}
	}
	catch(std::runtime_error& e)
	{
		ROS_ERROR("Hid Error: %s", e.what());
	}
	catch(...)
	{
		// Ignore errors
	}
}

void HidIO::CheckSuccess(int rc, const char* message) const
{
	if (rc != LIBUSB_SUCCESS)
	{
		throw Error("%s: %s", message, libusb_error_name(rc));
	}
}

void HidIO::Read()
{
	if (IsOpen())
	{
		libusb_transfer* transfer = libusb_alloc_transfer(0);

		uint8_t* data = static_cast<uint8_t*>(calloc(rxMaxPacketSize_, sizeof(uint8_t)));

		libusb_fill_interrupt_transfer(transfer, deviceHandle_,
			rxEndpointAddress_, data, rxMaxPacketSize_, &HidIO::ReadCompleted, this, 0);

		SubmitTransfer(transfer);
	}
}

void HidIO::RegisterHotPlug()
{
	int rc = libusb_hotplug_register_callback(nullptr,
		(libusb_hotplug_event)(LIBUSB_HOTPLUG_EVENT_DEVICE_ARRIVED|LIBUSB_HOTPLUG_EVENT_DEVICE_LEFT),
		LIBUSB_HOTPLUG_ENUMERATE,
		pid_,
		vid_,
		LIBUSB_HOTPLUG_MATCH_ANY,
		&HidIO::HotPlugCallback,
		static_cast<void*>(this),
		&hotplugHandle_);

	CheckSuccess(rc, "Failed to register for hot plug events");
}

int HidIO::HotPlugCallback(libusb_context* ctx, libusb_device* device,
	libusb_hotplug_event event, void* userData)
{
	HidIO* pThis = reinterpret_cast<HidIO*>(userData);

	ExecuteInRosThread(std::bind(&HidIO::HotPlugCallbackInternal, pThis, ctx, device, event));

	return 0;
}

int HidIO::HotPlugCallbackInternal(libusb_context* ctx, libusb_device* device,
	libusb_hotplug_event event)
{
	try
	{
		if (event == LIBUSB_HOTPLUG_EVENT_DEVICE_ARRIVED)
		{
			ClaimDevice(device);
		}
	}
	catch(std::exception& e)
	{
		ROS_ERROR("Hid exception: %s", e.what());
	}
	catch(...)
	{
		ROS_ERROR("Hid unknown error");
	}

	return 0;
}

void HidIO::ClaimDevice(libusb_device* device)
{
	struct libusb_device_descriptor desrc = { 0 };

	int rc = libusb_open(device, &deviceHandle_);

	CheckSuccess(rc, "Failed to open usb device");

	// Detach the device from the kernel driver if it has claimed the device
	if (libusb_kernel_driver_active(deviceHandle_, 0) == 1)
	{
		rc = libusb_detach_kernel_driver(deviceHandle_, 0);

		CheckSuccess(rc, "Failed to detach usb device from kernel");
	}

	rc = libusb_claim_interface(deviceHandle_, 0);

	CheckSuccess(rc, "Failed to claim usb device");

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

	LookupEndpoints();
}

void HidIO::ReleaseDevice()
{
	if (deviceHandle_)
	{
		int rc = libusb_release_interface(deviceHandle_, 0);

		CheckSuccess(rc, "Failed to release usb device");

		libusb_close(deviceHandle_);
		deviceHandle_ = nullptr;
	}

	product_ = "";
	vendor_ = "";
}

void HidIO::LookupEndpoints()
{
	rxEndpointAddress_	= -1;
	rxMaxPacketSize_	= -1;
	txEndpointAddress_	= -1;
	txMaxPacketSize_	= -1;

	libusb_device* device = libusb_get_device(deviceHandle_);

	libusb_config_descriptor* config = nullptr;
	int rc = libusb_get_active_config_descriptor(device, &config);

	CheckSuccess(rc, "Failed to get active config descriptor");

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

	if (rxEndpointAddress_	== -1	||
		rxMaxPacketSize_	== -1	||
		txEndpointAddress_	== -1	||
		txMaxPacketSize_	== -1)
	{
		throw Error("Failed to get endpoint addresses and/or size");
	}

	ROS_INFO("USB device connected: pid=0x%x, vid=0x%x, vendor=%s, product=%s rxAddr=0x%x rxSize=%d txAddr=0x%x txSize=%d",
		pid_, vid_, vendor_.c_str(), product_.c_str(),
		rxEndpointAddress_, rxMaxPacketSize_, txEndpointAddress_, txMaxPacketSize_);

	connetionCallback_(true);

	Read();
}

void HidIO::ReadCompleted(libusb_transfer* transfer)
{
	HidIO* pThis = reinterpret_cast<HidIO*>(transfer->user_data);

	pThis->ReadCompletedInternal(transfer);
}

bool HidIO::HasTransfers()
{
	return transfers_.size() > 0;
}

void HidIO::SubmitTransfer(libusb_transfer* transfer)
{
	transfer->flags |= LIBUSB_TRANSFER_FREE_BUFFER;
	transfer->flags |= LIBUSB_TRANSFER_FREE_TRANSFER;
	transfer->flags |= LIBUSB_TRANSFER_SHORT_NOT_OK;

	int rc = libusb_submit_transfer(transfer);

	CheckSuccess(rc, "Hid submit transfer failed ");

	transfers_.insert(transfer);
}

void HidIO::CleanupTransfer(libusb_transfer* transfer)
{
	transfers_.erase(transfer);
}

void HidIO::ReadCompletedInternal(libusb_transfer* transfer)
{
	try
	{
		if (transfer->status == LIBUSB_TRANSFER_COMPLETED)
		{
			uint8_t size = transfer->buffer[0];

			std::vector<char> message(transfer->buffer + 1, transfer->buffer + 1 + size);

			if(readCallback_)
			{
//				ROS_ERROR_STREAM("ReadData (" << size << "): " <<
//					ToHex(std::vector<char>(message.begin(), message.end())));

				readCallback_(std::move(message));
			}
			else
			{
				printf("Hid IO read but no callback specified!\n");
			}
		}
		else
		{
			ROS_ERROR("Hid read failed: %d", transfer->status);
		}
	}
	catch(std::exception& e)
	{
		ROS_ERROR("Hid exception: %s", e.what());
	}
	catch(...)
	{

	}

	CleanupTransfer(transfer);

	if (transfer->status == LIBUSB_TRANSFER_NO_DEVICE)
	{
		Close();
	}
	else
	{
		Read();
	}
}

void HidIO::WriteCompleted(libusb_transfer* transfer)
{
	HidIO* pThis = reinterpret_cast<HidIO*>(transfer->user_data);

	pThis->WriteCompletedInternal(transfer);
}

void HidIO::WriteCompletedInternal(libusb_transfer* transfer)
{
	try
	{
		if (transfer->status != LIBUSB_TRANSFER_COMPLETED)
		{
			ROS_ERROR("Hid write failed: %d", transfer->status);
		}
	}
	catch(std::exception& e)
	{
		ROS_ERROR("Hid exception: %s", e.what());
	}
	catch(...)
	{

	}

	CleanupTransfer(transfer);

	if (transfer->status == LIBUSB_TRANSFER_NO_DEVICE)
	{
		Close();
	}
}

}
