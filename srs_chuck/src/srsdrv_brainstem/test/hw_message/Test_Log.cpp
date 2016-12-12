/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */

#include <srsdrv_brainstem/hw_message/LogHandler.hpp>
#include <hw_message/Test_HardwareMessage.hpp>

struct LogData
{
	uint8_t level;
	std::string message;
};

class Test_Log : public Test_HardwareMessage<const LogData&, LogData>
{
public:

	Test_Log() :
		Test_HardwareMessage(),
		level_(LOG_LEVEL::UNKNOWN),
		message_()
	{
		logCallback_ = std::bind(&Test_Log::logCallback, this, std::placeholders::_1,
			std::placeholders::_2);
	}

	virtual ~Test_Log() {}

	vector<LogData>& getTestMsgs()
	{
		static vector<LogData> testMessages;

		if (!testMessages.size())
		{
			LogData log;
			log.level = 0;
			log.message = "Debug message";
			testMessages.push_back(log);
		}

		return testMessages;
	}

	vector<char> createPacket(const LogData& log, uint32_t version = 1)
	{
		vector<char> messageBuffer;

		HardwareMessage msg(messageBuffer);

		msg.write<uint8_t>(static_cast<uint8_t>(BRAIN_STEM_MSG::LOG));
		msg.write<uint8_t>(log.level);
		msg.write(log.message);

		return std::move(messageBuffer);
	}

	void logCallback(LOG_LEVEL level, std::string message)
	{
		level_ = level;

		message_ = message;
	}


	LOG_LEVEL level_;

	std::string message_;

	LogHandler::LogCallbackFn logCallback_;
};


namespace srslib_framework
{
	bool operator== (const LogData& state1,
		const LogData& state2)
	{
		return state1.level == state2.level &&
			state1.message == state2.message;
	}
}

TEST_F(Test_Log, Key)
{
	LogHandler LogHandler(logCallback_);

    ASSERT_EQ(LogHandler.getKey(), BRAIN_STEM_MSG::LOG);
}

TEST_F(Test_Log, EmptyPacket)
{
	LogHandler LogHandler(logCallback_);

	vector<char> messageBuffer;

	try
	{
		LogHandler.receiveData(ros::Time(), messageBuffer);

		ASSERT_FALSE("Parsed invalid (empty) Rpm packet");
	}
	catch(std::runtime_error&)
	{
		// Success
	}
}

TEST_F(Test_Log, ValidMessages)
{
	LogHandler logHandler(logCallback_);

	vector<LogData>& testMsgs = getTestMsgs();

	for (auto msg : testMsgs)
	{
		vector<char> messageBuffer = createPacket(testMsgs[0]);

		// Setup test expectations
		logHandler.receiveData(ros::Time(), messageBuffer);

		EXPECT_EQ(level_, msg.level);
		EXPECT_EQ(message_, msg.message);
	}
}
