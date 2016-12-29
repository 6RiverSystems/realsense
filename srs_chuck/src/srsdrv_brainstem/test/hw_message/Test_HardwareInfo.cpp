/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#include <hw_message/HardwareInfoHandler.hpp>
#include <hw_message/Test_HardwareMessage.hpp>

class Test_HardwareInfo : public Test_HardwareMessage<const MsgHardwareInfo&, MsgHardwareInfo>
{
public:

	Test_HardwareInfo() :
		Test_HardwareMessage() {}

	virtual ~Test_HardwareInfo() {}

	vector<MsgHardwareInfo>& getTestMsgs()
	{
		static vector<MsgHardwareInfo> testMessages;

		if (!testMessages.size())
		{
			MsgHardwareInfo hardwareInfo;
			hardwareInfo.name = "not in protocol - no need to test here";
			hardwareInfo.uid = "31005335-4B50-5948-3331-303030323333";
			hardwareInfo.chassisGeneration = 1;
			hardwareInfo.brainstemHwVersion = 2;
			hardwareInfo.brainstemSwVersion = "EP1/0.0.7-43-g29263b5-dirty";

			MsgBatteryInfo batteryInfo;
			batteryInfo.manufacturer.reserve(13);
			batteryInfo.manufacturer.reserve(33);
			batteryInfo.manufacturer = "012345678912"; // 12 chars + 1 null terminator (limit)
			batteryInfo.serialNumber = "01234567890123456789012345678912"; // 32 chars + 1 null terminator (limit)
			hardwareInfo.batteryInfo.push_back(batteryInfo);
			hardwareInfo.batteryInfo.push_back(batteryInfo);

			testMessages.push_back(hardwareInfo);
		}

		return testMessages;
	}

	vector<char> createPacket(const MsgHardwareInfo& hardwareInfo, uint32_t version = 1)
	{
		vector<char> messageBuffer;

		HardwareMessage msg(messageBuffer);

		msg.write<uint8_t>(static_cast<uint8_t>(BRAIN_STEM_MSG::HARDWARE_INFO));

		uint16_t uid[8] = { 0 };
		uint32_t uidScan[8] = { 0 };

		sscanf(hardwareInfo.uid.c_str(), "%04X%04X-%04X-%04X-%04X-%04X%04X%04X",
			&uidScan[0], &uidScan[1], &uidScan[2], &uidScan[3], &uidScan[4], &uidScan[5], &uidScan[6], &uidScan[7]);

		for (int i = 0; i < 8; i++)
		{
			uid[i] = static_cast<uint16_t>(uidScan[i]);
		}

		msg.writeArray<uint16_t>(uid, 8);
		msg.write<uint8_t>(hardwareInfo.chassisGeneration);
		msg.write<uint8_t>(hardwareInfo.brainstemHwVersion);
		msg.write(hardwareInfo.brainstemSwVersion);

		if (version == 2)
		{
			uint8_t numberOfBatteries = hardwareInfo.batteryInfo.size();

			msg.write<uint8_t>(numberOfBatteries);

			for (auto batteryInfo : hardwareInfo.batteryInfo)
			{
				msg.writeArray<const char>(batteryInfo.manufacturer.c_str(), 13);
				msg.writeArray<const char>(batteryInfo.serialNumber.c_str(), 33);
			}
		}

		return std::move(messageBuffer);
	}
};

namespace srslib_framework
{
	bool operator== (const MsgBatteryInfo& state1,
		const MsgBatteryInfo& state2)
	{
		return state1.manufacturer == state2.manufacturer &&
			state1.serialNumber == state2.serialNumber;
	}

	bool operator== (const MsgHardwareInfo& state1,
		const MsgHardwareInfo& state2)
	{
		return state1.uid == state2.uid &&
			state1.chassisGeneration == state2.chassisGeneration &&
			state1.brainstemHwVersion == state2.brainstemHwVersion &&
			state1.brainstemSwVersion == state2.brainstemSwVersion &&
			state1.batteryInfo == state2.batteryInfo;
	}
}

TEST_F(Test_HardwareInfo, Key)
{
	HardwareInfoHandler HardwareInfoHandler(nullptr, publisher_);

    ASSERT_EQ(HardwareInfoHandler.getKey(), BRAIN_STEM_MSG::HARDWARE_INFO);
}

TEST_F(Test_HardwareInfo, EmptyPacket)
{
	HardwareInfoHandler HardwareInfoHandler(nullptr, publisher_);

	vector<char> messageBuffer;

	try
	{
		HardwareInfoHandler.receiveData(ros::Time(), messageBuffer);
	}
	catch(std::runtime_error&)
	{
		// Success
	}
}

TEST_F(Test_HardwareInfo, BadPacket)
{
	HardwareInfoHandler hardwareInfoHandler(nullptr, publisher_);

	vector<MsgHardwareInfo>& testMsgs = getTestMsgs();

	vector<char> messageBuffer = createPacket(testMsgs[0]);
	messageBuffer.pop_back();

	try
	{
		hardwareInfoHandler.receiveData(ros::Time(), messageBuffer);
	}
	catch(std::runtime_error&)
	{
		// Success
	}
}

TEST_F(Test_HardwareInfo, ValidMessage)
{
	HardwareInfoHandler hardwareInfoHandler(nullptr, publisher_);

	vector<MsgHardwareInfo>& testMsgs = getTestMsgs();

	for (auto msg : testMsgs)
	{
		vector<char> messageBuffer = createPacket(msg);

		hardwareInfoHandler.receiveData(ros::Time(), messageBuffer);

		publisher_.data_.batteryInfo = msg.batteryInfo;

		// Copy value since version one did not have battery info
		EXPECT_EQ(publisher_.data_, msg);
	}
}

TEST_F(Test_HardwareInfo, BatteryInfo)
{
	HardwareInfoHandler hardwareInfoHandler(nullptr, publisher_);

	vector<MsgHardwareInfo>& testMsgs = getTestMsgs();

	for (auto msg : testMsgs)
	{
		vector<char> messageBuffer = createPacket(testMsgs[0], 2);

		hardwareInfoHandler.receiveData(ros::Time(), messageBuffer);

		EXPECT_EQ(publisher_.data_, msg);
	}
}
