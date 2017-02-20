#include <hw_message/HardwareInfoHandler.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

HardwareInfoHandler::HardwareInfoHandler(ChannelBrainstemHardwareInfo::Interface& publisher) :
    HardwareMessageHandler(BRAIN_STEM_MSG::HARDWARE_INFO),
	publisher_(publisher),
	hasValidMessage_(false)
{
	hardwareInfoMsg_.name = "";
    char* robotName = getenv("ROBOT_NAME");
    if (robotName)
    {
    	hardwareInfoMsg_.name = string(robotName);
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void HardwareInfoHandler::receiveMessage(ros::Time currentTime, HardwareMessage& msg)
{
	std::stringstream batteryStream;

	if (msg.checkBufferSize<HardwareInfoData>())
	{
		HardwareInfoData hwInfo = msg.read<HardwareInfoData>();

		char rawUid[255];
		sprintf(rawUid, "%04X%04X-%04X-%04X-%04X-%04X%04X%04X",
			hwInfo.uniqueId[0], hwInfo.uniqueId[1],
			hwInfo.uniqueId[2],
			hwInfo.uniqueId[3],
			hwInfo.uniqueId[4],
			hwInfo.uniqueId[5], hwInfo.uniqueId[6], hwInfo.uniqueId[7]);
		hardwareInfoMsg_.uid = string(rawUid);

		hardwareInfoMsg_.chassisGeneration = hwInfo.chassisGeneration;
		hardwareInfoMsg_.brainstemHwVersion = hwInfo.brainstemHwVersion;

		hardwareInfoMsg_.brainstemSwVersion = msg.readString();
	}

	hardwareInfoMsg_.batteryInfo.erase(hardwareInfoMsg_.batteryInfo.begin(), hardwareInfoMsg_.batteryInfo.end());

	if (msg.checkBufferSize<uint8_t>())
	{
		uint8_t numberOfBatteries = msg.read<uint8_t>();

		ROS_INFO_STREAM("Number of batteries: " << (int)numberOfBatteries);

		for (int i = 0; i < numberOfBatteries; i++)
		{
			BatteryInfoData batteryInfo = msg.read<BatteryInfoData>();

			srslib_framework::MsgBatteryInfo batteryInfoMsg;
			batteryInfoMsg.manufacturer = batteryInfo.manufacturer;
			batteryInfoMsg.serialNumber = batteryInfo.serialNumber;

			hardwareInfoMsg_.batteryInfo.push_back(batteryInfoMsg);

			batteryStream << ", battery" << i << ": manufacturer=" << batteryInfoMsg.manufacturer <<
				", serial#=" << batteryInfoMsg.serialNumber;
		}
	}

	ROS_INFO_STREAM("Hardware Info {" <<
		"name: " << hardwareInfoMsg_.name <<
		", uid: " << hardwareInfoMsg_.uid <<
		", chassis generation: " << (int)hardwareInfoMsg_.chassisGeneration <<
		", brainstem hw version: " << (int)hardwareInfoMsg_.brainstemHwVersion <<
		", brainstem sw version: " << hardwareInfoMsg_.brainstemSwVersion <<
		batteryStream.str() << "}");

	// Publish the hardware info
	publisher_.publish(hardwareInfoMsg_);

	hasValidMessage_ = true;
}

} // namespace srs
