/*
 * HidIO.h
 *
 *  Created on: Apr 4, 2016
 *      Author: dan
 */
#include <boost/asio.hpp>
#include <boost/assert.hpp>
#include <functional>
#include <stdexcept>
#include <thread>
#include <set>
#include <future>
#include <mutex>

#include <libusb-1.0/libusb.h>

#include <srslib_framework/io/IO.hpp>

#pragma once

namespace srs {

class HidIO :
	public IO,
	private boost::noncopyable
{

public:

	HidIO(const char* pszName, int32_t pid, int32_t vid);

	virtual ~HidIO();

	void open(ConnectionCallbackFn connectionCallback,
		ReadCallbackFn readCallback);

	bool isOpen() const;

	void close();

	void spinOnce();

// Write Methods

	void write(const std::vector<char>& buffer);

private:

	bool checkSuccess(const char* action, int rc);

	void read();

	void registerHotPlug();

	static int hotPlugCallback(libusb_context *ctx, libusb_device *device,
		libusb_hotplug_event event, void *user_data);

	int hotPlugCallbackInternal(libusb_context *ctx, libusb_device *device,
		libusb_hotplug_event event);

	void claimDevice(libusb_device* device);

	void releaseDevice();

	int lookupEndpoints();

	bool hasTransfers();

	void submitTransfer(libusb_transfer* transfer);

	bool cleanupTransfer(libusb_transfer* transfer);

	static void readCompleted(libusb_transfer* transfer);

	void readCompletedInternal(libusb_transfer* transfer);

	static void writeCompleted(libusb_transfer* transfer);

	void writeCompletedInternal(libusb_transfer* transfer);

	bool									initializedUsb_;

	bool									claimedUsb_;

	std::string								name_;

    int32_t									pid_;

    int32_t									vid_;

	std::string								vendor_;

	std::string								product_;

	bool									open_;
	libusb_device_handle*					deviceHandle_;

	libusb_hotplug_callback_handle			hotplugHandle_;

	uint8_t									sequence_;

    uint8_t									rxEndpointAddress_;
    uint16_t								rxMaxPacketSize_;

    uint8_t									txEndpointAddress_;
    uint16_t								txMaxPacketSize_;

	std::set<libusb_transfer*>				transfers_;

    std::shared_ptr<std::thread>			thread_;

    ConnectionCallbackFn					connetionCallback_;

    ReadCallbackFn							readCallback_;

};

} /* namespace srs */
