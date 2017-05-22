/*
 * SerialIO.h
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

#include <srslib_framework/io/IO.hpp>

#pragma once

namespace srs {

class SerialIO :
	public IO,
	private boost::noncopyable
{
	typedef std::shared_ptr<boost::asio::deadline_timer> ConnectionTimer;

	enum class READ_STATE
	{
		DEFAULT,
		IN_MESSAGE,
		IN_MESSAGE_ESCAPED
	};

public:

	SerialIO(const char* pszName, const char* pszSerialPort);

	virtual ~SerialIO();

	void open(ConnectionCallbackFn connectionCallback,
		ReadCallbackFn readCallback);

	bool isOpen() const;

	void close();

	void spinOnce() {};

// Serial IO Configuration

	void setSynced(bool synced);

	void setRetryTimeout(float fRetryTimeout);

	void enableCRC(bool bEnableCRC);

	void setLeadingCharacter(char cLeading);

	void setTerminatingCharacter(char cTerminating);

	void setEscapeCharacter(char cEscape);

	void setFirstByteDelay(std::chrono::microseconds firstByteDelay);

	std::chrono::microseconds getFirstByteDelay();

	void setByteDelay(std::chrono::microseconds byteDelay);

	std::chrono::microseconds getByteDelay();

// Write Methods

	void write(const std::vector<char>& buffer);

#if defined(ENABLE_TEST_FIXTURE)

	typedef std::vector<std::chrono::microseconds> MessageTiming;

	std::vector<MessageTiming> getTimingInfo() { return vecMessageTiming_; };

#endif

private:

	void writeInSerialThread(std::vector<char> writeBuffer);

	void startAsyncTimer();

	void startAsyncRead();

	void onWriteComplete(const boost::system::error_code& error, std::size_t size);

	void onReadComplete(const boost::system::error_code& error, std::size_t size);

// Connection Retry Logic

	bool onCheckSerialPort(bool bInitialCheck, const boost::system::error_code& e =
		boost::system::error_code());

private:

	std::string								name_{ "" };

	bool									isSynced_{ false };

	std::string								strDebug_{ "io." + name_ };

    std::shared_ptr<std::thread>			thread_{ };

    std::thread::id							serialThreadId_{ };

	boost::asio::io_service					ioService_{ };

	boost::asio::io_service::work			work_{ ioService_ };

	ConnectionTimer							timer_{ };

    boost::asio::serial_port				serialPort_{ ioService_ };

    bool									isSerialOpen_{ false };

	std::string								strSerialPort_{ };

	float									retryTimeout_{ 1.0f };

    bool									isWriting_{ false };

    std::vector<char>						readBuffer_{ std::vector<char>( 1024 ) };

    std::vector<char>						writeBuffer_{ };

    std::vector<char>						message_{ };

	READ_STATE								readState_{ READ_STATE::DEFAULT };

	uint8_t									cCRC_{ 0 };

    ConnectionCallbackFn					connectionCallback_{ };

    ReadCallbackFn							readCallback_{ };

	bool									enableCRC_{ false };

	bool									hasLeading_{ false };

	char									leading_{ 0 };

	bool									hasTerminating_{ false };

	char									terminating_{ 0 };

	bool									hasEscape_{ false };

	char									cEscape_{ 0 };

	std::chrono::microseconds				firstByteDelay_{ };

	std::chrono::microseconds				byteDelay_{ };

	std::chrono::microseconds				interByteDelay_{ };

#if defined(ENABLE_TEST_FIXTURE)

	std::chrono::high_resolution_clock				highrezclk_{ };

	std::chrono::high_resolution_clock::time_point	lastTime_{ std::chrono::milliseconds(0) };

	MessageTiming									messageTiming_{ };

	std::vector<MessageTiming>						vecMessageTiming_{ };

#endif

};

} /* namespace srs */
