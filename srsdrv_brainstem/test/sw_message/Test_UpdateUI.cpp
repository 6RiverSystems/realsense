/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */

#include <sw_message/Test_SoftwareMessage.hpp>
#include <sw_message/UpdateUIHandler.hpp>

namespace srs {

class Test_UpdateUI :
    public Test_SoftwareMessage<srslib_framework::MsgUpdateUI>
{
public:
    virtual ~Test_UpdateUI() {}

	vector<srslib_framework::MsgUpdateUI>& getTestMsgs()
	{
		static vector<srslib_framework::MsgUpdateUI> testMessages;

		if (!testMessages.size())
		{
			srslib_framework::MsgUpdateUI updateUI;

			srslib_framework::MsgUIState uiState;
			uiState.element = static_cast<uint8_t>(LED_ENTITIES::TOTE0);
			uiState.mode = static_cast<uint8_t>(LED_MODE::GRAB);

			updateUI.uiUpdates.push_back(uiState);

			testMessages.push_back(updateUI);
		}

		return testMessages;
	}

	vector<char> createPacket(const srslib_framework::MsgUpdateUI& msg)
	{
		vector<char> messageBuffer;

		HardwareMessage hardwareMessage(messageBuffer);

		for (auto uiElement : msg.uiUpdates)
		{
			hardwareMessage.write<uint8_t>(static_cast<uint8_t>(BRAIN_STEM_CMD::UPDATE_LIGHT));
			hardwareMessage.write<uint8_t>(uiElement.element);
			hardwareMessage.write<uint8_t>(uiElement.mode);

			return std::move(messageBuffer);
		}
	}
};


TEST_F(Test_UpdateUI, TestEncodeData)
{
	UpdateUIHandler UpdateUIHandler(&messageProcessor_);

	vector<srslib_framework::MsgUpdateUI>& testMsgs = getTestMsgs();

	for (auto msg : testMsgs)
	{
		UpdateUIHandler.encodeData(msg);

		std::vector<char> message = createPacket(msg);

		ASSERT_EQ(messageProcessor_.command_, message);
	}
}

} // namespace srs
