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

	void Open(ConnectionCallbackFn connectionCallback,
		ReadCallbackFn readCallback);

	bool IsOpen() const;

	void Close();

// Write Methods

	void Write(const std::vector<char>& buffer);

private:

	void CheckSuccess(int rc, const char* message) const;

	void Read();

	void RegisterHotPlug();

	static int HotPlugCallback(libusb_context *ctx, libusb_device *device,
		libusb_hotplug_event event, void *user_data);

	int HotPlugCallbackInternal(libusb_context *ctx, libusb_device *device,
		libusb_hotplug_event event);

	void ClaimDevice(libusb_device* device);

	void ReleaseDevice();

	void LookupEndpoints();

	void SubmitTransfer(libusb_transfer* transfer);

	static void ReadCompleted(libusb_transfer* transfer);

	void ReadCompletedInternal(libusb_transfer* transfer);

	static void WriteCompleted(libusb_transfer* transfer);

	void WriteCompletedInternal(libusb_transfer* transfer);

	bool									runEventThread_;

	std::string								name_;

    int32_t									pid_;

    int32_t									vid_;

	std::string								vendor_;

	std::string								product_;

	bool									open_;
	libusb_device_handle*					deviceHandle_;

	libusb_hotplug_callback_handle			hotplugHandle_;

    uint8_t									rxEndpointAddress_;
    uint16_t								rxMaxPacketSize_;

    uint8_t									txEndpointAddress_;
    uint16_t								txMaxPacketSize_;

    std::mutex								transferMutex_;
	std::set<libusb_transfer*>				transfers_;

    std::shared_ptr<std::thread>			thread_;

    ConnectionCallbackFn					connetionCallback_;

    ReadCallbackFn							readCallback_;

};

} /* namespace srs */
