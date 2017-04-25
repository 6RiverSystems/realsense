/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */

#include <hw_message/ButtonPressedHandler.hpp>

#include <hw_message/Test_HardwareMessage.hpp>

class Test_ButtonPressed : public Test_HardwareMessage<const uint32_t&, std_msgs::UInt32>
{
public:

	Test_ButtonPressed() :
		Test_HardwareMessage([](uint32_t data) {
		std_msgs::UInt32 msg;
		msg.data = data;
		return msg;
	}) {}

	virtual ~Test_ButtonPressed() {}

	vector<std_msgs::UInt32>& getTestMsgs()
	{
		static vector<std_msgs::UInt32> testMessages;

		if (!testMessages.size())
		{
			std_msgs::UInt32 button;
			button.data = 1;

			testMessages.push_back(button);
		}

		return testMessages;
	}

	vector<char> createPacket(const std_msgs::UInt32& buttonPressed, uint32_t version = 1)
	{
		vector<char> messageBuffer;

		HardwareMessage msg(messageBuffer);

		msg.write<uint8_t>(static_cast<uint8_t>(BRAIN_STEM_MSG::BUTTON_PRESSED));
		msg.write<uint8_t>(buttonPressed.data);

		return std::move(messageBuffer);
	}

};

TEST_F(Test_ButtonPressed, Key)
{
	ButtonPressedHandler ButtonPressedHandler(publisher_);

    ASSERT_EQ(ButtonPressedHandler.getKey(), BRAIN_STEM_MSG::BUTTON_PRESSED);
}

TEST_F(Test_ButtonPressed, EmptyPacket)
{
	ButtonPressedHandler ButtonPressedHandler(publisher_);

	vector<char> messageBuffer;

	try
	{
		ButtonPressedHandler.receiveData(ros::Time(), messageBuffer);

		ASSERT_FALSE("Parsed invalid (empty) packet");
	}
	catch(std::runtime_error&)
	{
		// Success
	}
}

TEST_F(Test_ButtonPressed, BadPacket)
{
	ButtonPressedHandler buttonPressedHandler(publisher_);

	vector<std_msgs::UInt32>& testMsgs = getTestMsgs();

	vector<char> messageBuffer = createPacket(testMsgs[0]);
	messageBuffer.pop_back();

	try
	{
		buttonPressedHandler.receiveData(ros::Time(), messageBuffer);

		ASSERT_FALSE("Parsed invalid (badly formed) Rpm packet");
	}
	catch(std::runtime_error&)
	{
		// Success
	}
}

TEST_F(Test_ButtonPressed, ValidMessages)
{
	ButtonPressedHandler buttonPressedHandler(publisher_);

	vector<std_msgs::UInt32>& testMsgs = getTestMsgs();

	for (auto msg : testMsgs)
	{
		vector<char> messageBuffer = createPacket(testMsgs[0]);

		// Setup test expectations
		buttonPressedHandler.receiveData(ros::Time(), messageBuffer);

		EXPECT_EQ(publisher_.data_.data, msg.data);
	}
}
