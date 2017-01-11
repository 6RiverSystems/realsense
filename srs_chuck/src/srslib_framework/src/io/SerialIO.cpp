/*
 * SerialIO.cpp
 *
 *  Created on: Apr 4, 2016
 *      Author: dan
 */

#include <srslib_framework/io/SerialIO.hpp>
#include <srslib_framework/utils/Logging.hpp>
#include <srslib_framework/platform/Thread.hpp>
#include <thread>
#include <ros/ros.h>

namespace srs
{

SerialIO::SerialIO(const char* pszName, const char* pszSerialPort) :
	name_(pszName),
	strSerialPort_(pszSerialPort),
	isSynced_(false),
	strDebug_("io." + name_),
	thread_(),
	serialThreadId_(),
	work_(ioService_),
	serialPort_(ioService_),
	isSerialOpen_(false),
	retryTimeout_(1.0),
	isWriting_(false),
	readBuffer_(1024),
	writeBuffer_(),
	message_(),
	readState_(READ_STATE::DEFAULT),
	cCRC_(0),
	readCallback_(),
	enableCRC_(false),
	hasLeading_(false),
	leading_(0),
	hasTerminating_(false),
	terminating_(0),
	hasEscape_(false),
	cEscape_(0),
	firstByteDelay_(),
	byteDelay_(),
	interByteDelay_()
#if defined(ENABLE_TEST_FIXTURE)
	,
	highrezclk_(),
	lastTime_(std::chrono::milliseconds(0)),
	messageTiming_(),
	vecMessageTiming_()
#endif
{
	timer_.reset(new boost::asio::deadline_timer(ioService_));

	// Spin up the thread
	thread_.reset(new std::thread([&]()
	{
		ioService_.run();
	}));
}

SerialIO::~SerialIO()
{
	close();

	// Stop the service (and the thread)
	ioService_.stop();

	// Clean up the thread
	thread_->join();
}

void SerialIO::open(ConnectionCallbackFn connectionCallback,
	ReadCallbackFn readCallback)
{
	if(!isOpen())
	{
		isSynced_ = false;

		std::condition_variable condition;

		std::mutex mutex;

		ioService_.post([&]()
			{
				connectionCallback_ = connectionCallback;

				readCallback_ = readCallback;

				serialThreadId_ = std::this_thread::get_id();

				onCheckSerialPort(false);

				condition.notify_one();
			});


		std::unique_lock<std::mutex> lock(mutex);

		condition.wait(lock);
	}
	else
	{
		ROS_ERROR_NAMED(strDebug_, "%s: Attempt to open serial port but it is already open.",
			name_.c_str());
	}
}

bool SerialIO::isOpen() const
{
	return serialPort_.is_open();
}

void SerialIO::close()
{
	if(serialPort_.is_open())
	{
		serialPort_.close();

		isSynced_ = false;
	}
}

////////////////////////////////////////////////////////////////////////////////////////////
// Serial IO Configuration
////////////////////////////////////////////////////////////////////////////////////////////

void SerialIO::setSynced(bool synced)
{
	isSynced_ = synced;
}

void SerialIO::setRetryTimeout(float fRetryTimeout)
{
	retryTimeout_ = fRetryTimeout;
}

void SerialIO::enableCRC(bool bGenerateCRC)
{
	enableCRC_ = bGenerateCRC;
}

void SerialIO::setLeadingCharacter(char cLeading)
{
	leading_ = cLeading;

	hasLeading_ = true;
}

void SerialIO::setTerminatingCharacter(char cTerminating)
{
	terminating_ = cTerminating;

	hasTerminating_ = true;
}

void SerialIO::setEscapeCharacter(char cEscape)
{
	cEscape_ = cEscape;

	hasEscape_ = true;
}

void SerialIO::setFirstByteDelay(std::chrono::microseconds firstByteDelay)
{
	firstByteDelay_ = firstByteDelay;
}

std::chrono::microseconds SerialIO::getFirstByteDelay()
{
	return firstByteDelay_;
}

void SerialIO::setByteDelay(std::chrono::microseconds byteDelay)
{
	byteDelay_ = byteDelay;
}

std::chrono::microseconds SerialIO::getByteDelay()
{
	return byteDelay_;
}

void SerialIO::write(const std::vector<char>& buffer)
{
//	ROS_DEBUG_STREAM_NAMED(m_strDebug, "%s: Write: " << m_strName.c_str(), ToHex(buffer));

	if(isOpen())
	{
		// Do all data operations in serial processing thread
		ioService_.post(std::bind(&SerialIO::writeInSerialThread, this, buffer));
	}
	else
	{
		// Throw error
		throw std::runtime_error("port not open");
	}
}

void SerialIO::writeInSerialThread(std::vector<char> writeBuffer)
{
//	ROS_DEBUG_STREAM_NAMED(m_strDebug, m_strName << ": WriteInSerialThread: " <<
//		ToHex(writeBuffer));

	assert(serialThreadId_ == std::this_thread::get_id());

	// Add leading character
	if(hasLeading_)
	{
		writeBuffer_.push_back(leading_);
	}

	uint8_t cCRC = 0;

	// Function to add an escaped char (if necessary) to the array
	auto addEscapedCharacter =
		[&](const char& cChar)
		{
			if(hasEscape_)
			{
				if((cChar == cEscape_)					 ||
					(hasLeading_ && cChar == leading_) ||
					(hasTerminating_ && cChar == terminating_))
				{
					writeBuffer_.push_back(cEscape_);
				}
			}

			writeBuffer_.push_back(cChar);
		};

	for(const char& cChar : writeBuffer)
	{
		cCRC += cChar;

		addEscapedCharacter(cChar);
	}

	// Add CRC
	if(enableCRC_)
	{
		addEscapedCharacter(-cCRC);
	}

	// Add terminating character
	if(hasTerminating_)
	{
		writeBuffer_.push_back(terminating_);
	}

	// If we are already trying to write, then wait until we receive the callback
	if(!isWriting_)
	{
		isWriting_ = true;

		interByteDelay_ = firstByteDelay_;

		// If we have a first byte delay only write one byte
		size_t writeSize = interByteDelay_.count() ?  1 : writeBuffer_.size();

		ROS_DEBUG_NAMED(strDebug_, "%s: Serial Write: %s", name_.c_str(),
			ToHex(writeBuffer_).c_str());

		serialPort_.async_write_some(boost::asio::buffer(writeBuffer_.data(), writeSize),
			std::bind(&SerialIO::onWriteComplete, this, std::placeholders::_1, std::placeholders::_2));
	}
}

void SerialIO::startAsyncTimer()
{
	timer_->cancel();

	timer_->expires_from_now(boost::posix_time::seconds(retryTimeout_));

	timer_->async_wait([&](const boost::system::error_code& e)
		{
			onCheckSerialPort(false, e);
		});
}

void SerialIO::startAsyncRead()
{
	serialPort_.async_read_some(boost::asio::buffer(readBuffer_, readBuffer_.size()),
		std::bind(&SerialIO::onReadComplete, this, std::placeholders::_1, std::placeholders::_2));
}

void SerialIO::onWriteComplete(const boost::system::error_code& error, std::size_t size)
{
	assert(serialThreadId_ == std::this_thread::get_id());

//	ROS_DEBUG_STREAM_NAMED(m_strDebug, m_strName << ": Write Complete (" << size << ", " << m_writeBuffer.size() << "):" <<
//		ToHex(std::vector<char>(m_writeBuffer.begin(), m_writeBuffer.begin() + size)));

	std::vector<char>(writeBuffer_.begin() + size, writeBuffer_.end()).swap(writeBuffer_);

//	ROS_DEBUG_STREAM_NAMED(m_strDebug, m_strName << ": Write Complete remaining (" << m_writeBuffer.size() << "):" <<
//		ToHex(m_writeBuffer));

	if(writeBuffer_.begin() != writeBuffer_.end())
	{
//		ROS_DEBUG_STREAM_NAMED(m_strDebug, "Write Bytes: sleep for " << m_interByteDelay.count());

		std::this_thread::sleep_for(interByteDelay_);

		interByteDelay_ = byteDelay_;

		size_t writeSize = interByteDelay_.count() ? 1 : writeBuffer_.size();

		serialPort_.async_write_some(boost::asio::buffer(writeBuffer_.data(), writeSize),
			std::bind(&SerialIO::onWriteComplete, this, std::placeholders::_1, std::placeholders::_2));

		// Reset the first byte delay if we encounter a start or end of message
		if(size == 1 && firstByteDelay_.count())
		{
			if(hasLeading_)
			{
				if(leading_ == writeBuffer_[0])
				{
					interByteDelay_ = firstByteDelay_;
				}
			}
			else if(hasTerminating_)
			{
				if(terminating_ == writeBuffer_[0])
				{
					interByteDelay_ = firstByteDelay_;
				}
			}
		}
	}
	else
	{
		isWriting_ = false;
	}
}

void SerialIO::onReadComplete(const boost::system::error_code& error, std::size_t size)
{
	#if defined(ENABLE_TEST_FIXTURE)

		auto now = highrezclk_.now();

	#endif

	assert(serialThreadId_ == std::this_thread::get_id());

	if(!error)
	{
		// Start with the last parsed message
		// Add room for the new data
		message_.reserve(message_.size() + size);

		size_t messageStart = 0;

		auto changeState =
			[&](const READ_STATE& eNewState)
			{
				#if defined(ENABLE_TEST_FIXTURE)
					if(readState_ == READ_STATE::DEFAULT &&
						eNewState == READ_STATE::IN_MESSAGE)
					{
//						ROS_DEBUG_NAMED(m_strDebug, "Time reset");

						lastTime_ = now;
					}
				#endif

				readState_ = eNewState;
			};

		// Function to add an escaped char (if necessary) to the array
		auto addCharacter =
			[&](const char& cChar)
			{
				message_.push_back(cChar);

				cCRC_ += cChar;
			};

		for(int i = 0; i < size; i++)
		{
			#if defined(ENABLE_TEST_FIXTURE)

				ROS_DEBUG_NAMED(strDebug_, "Time between reads %zu: %zu ", message_.size(), std::chrono::duration_cast<std::chrono::microseconds>(now - lastTime_).count());

				messageTiming_.push_back(std::chrono::duration_cast<std::chrono::microseconds>(now - lastTime_));

				lastTime_ = now;

			#endif

			// If we do not have a leading character, then we are always in a message
			if(readState_ == READ_STATE::DEFAULT && !hasLeading_)
			{
				changeState(READ_STATE::IN_MESSAGE);
			}

//			ROS_ERROR_NAMED(m_strDebug, "Char: %02x", (unsigned char)m_readBuffer[i]);

			if(hasEscape_ &&
				readBuffer_[i] == cEscape_)
			{
				if(readState_ == READ_STATE::IN_MESSAGE_ESCAPED)
				{
					addCharacter(readBuffer_[i]);

					changeState(READ_STATE::IN_MESSAGE);
				}
				else
				{
					changeState(READ_STATE::IN_MESSAGE_ESCAPED);
				}
			}
			else if(readState_ == READ_STATE::DEFAULT)
			{
				// Should never get into this case if we are not a leading character
				assert(hasLeading_);

				// Only start a message when we encounter a leading character
				if(readBuffer_[i] == leading_)
				{
					changeState(READ_STATE::IN_MESSAGE);
				}
			}
			else if(readState_ == READ_STATE::IN_MESSAGE_ESCAPED ||
					readBuffer_[i] != terminating_)
			{
				addCharacter(readBuffer_[i]);

				changeState(READ_STATE::IN_MESSAGE);
			}
			else
			{
				if(message_.size() > 0)
				{
					if(message_[0] == '<')
					{
						cCRC_ = 0;
					}
					else if(enableCRC_)
					{
						// Don't remove the crc if we failed
						if(cCRC_ == 0)
						{
							message_.pop_back();
						}
					}
					else
					{
						// We are not using a CRC
						cCRC_ = 0;
					}

					if(cCRC_ == 0)
					{
						if(readCallback_)
						{
//							ROS_DEBUG_STREAM_NAMED(m_strDebug, "ReadData (" << messageSize_ << "): " <<
//								ToHex(std::vector<char>(message_.begin(), message_.begin() + messageSize_)));

							readCallback_(message_);

							message_ = std::vector<char>();
						}
						else
						{
							ROS_ERROR_NAMED(strDebug_, "Serial port data read but no callback specified!\n");
						}
					}
					else
					{
						if (isSynced_)
						{
							ROS_ERROR_STREAM_NAMED(strDebug_, "Invalid CRC (" << message_.size() << "): " <<
								ToHex(std::vector<char>(message_.begin(), message_.begin() + message_.size())) <<
								"(CRC: " << ToHex(std::vector<char>({ (char)cCRC_ })) << ")");
						}
					}
				}
				else
				{
					ROS_ERROR_STREAM_NAMED(strDebug_, "Empty Message");
				}

				cCRC_ = 0;

				changeState(READ_STATE::DEFAULT);

				#if defined(ENABLE_TEST_FIXTURE)

					vecMessageTiming_.push_back(messageTiming_);

					messageTiming_.clear();

				#endif
			}
		}
	}
	else
	{
		if(isOpen())
		{
			ROS_ERROR_NAMED(strDebug_, "Read Error: %s", error.message().c_str());

			if(error == boost::asio::error::eof)
			{
				close();
			}
		}
	}

	if(isOpen())
	{
		startAsyncRead();
	}
	else
	{
		startAsyncTimer();
	}
}

bool SerialIO::onCheckSerialPort(bool bInitialCheck, const boost::system::error_code& e)
{
	assert(serialThreadId_ == std::this_thread::get_id());

	// Save current state (so we don't spam debug log on reconnect)
	bool bIsSerialOpen = isSerialOpen_;

	isSerialOpen_ = isOpen();

	std::string strError;

	if(!isSerialOpen_)
	{
		try
		{
			// Anonymous function call the message processor in the main ros thread
			serialPort_.open(strSerialPort_);

			// Setup serial port for 8/N/1 operation @ 115.2kHz
			serialPort_.set_option(boost::asio::serial_port::baud_rate(115200));
			serialPort_.set_option(boost::asio::serial_port::flow_control(boost::asio::serial_port::flow_control::none));
			serialPort_.set_option(boost::asio::serial_port::character_size(8));
			serialPort_.set_option(boost::asio::serial_port::parity(boost::asio::serial_port::parity::none));
			serialPort_.set_option(boost::asio::serial_port::stop_bits(boost::asio::serial_port::stop_bits::one));

			readState_ = READ_STATE::DEFAULT;

			cCRC_ = 0;

			#if defined(ENABLE_TEST_FIXTURE)

				vecMessageTiming_.clear();

			#endif

			isSerialOpen_ = true;
		}
		catch(const std::exception& e)
		{
			strError = e.what();
		}
		catch(...)
		{
			strError = "Unknown error";
		}

		if(!isSerialOpen_ && bInitialCheck)
		{
			ROS_DEBUG_NAMED(strDebug_, "%s: Error connecting to serial port (%s): %s (Will retry every %.2f seconds)",
				name_.c_str(), strSerialPort_.c_str(), strError.c_str(), retryTimeout_);
		}
	}

	if(bIsSerialOpen != isSerialOpen_)
	{
		if(isSerialOpen_ == true)
		{
			ROS_INFO_NAMED(strDebug_, "%s: Connected to serial port: %s",
				name_.c_str(), strSerialPort_.c_str());
		}
		else
		{
			ROS_ERROR_NAMED(strDebug_, "%s: Disconnected from serial port (%s): %s (Will retry every %.2f seconds)",
				name_.c_str(), strSerialPort_.c_str(), strError.c_str(), retryTimeout_);
		}

		if(connectionCallback_)
		{
			connectionCallback_(isSerialOpen_);
		}
	}

	// If we failed to open the serial port keep trying
	if(isSerialOpen_)
	{
		startAsyncRead();
	}
	else
	{
		startAsyncTimer();
	}

	return isSerialOpen_;
}

} /* namespace srs */
