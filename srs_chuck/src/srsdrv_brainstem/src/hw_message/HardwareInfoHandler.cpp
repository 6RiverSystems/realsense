#include <srsdrv_brainstem/hw_message/HardwareInfoHandler.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

HardwareInfoHandler::HardwareInfoHandler(ChannelBrainstemHardwareInfo channel) :
    HardwareMessageHandler(BRAIN_STEM_MSG::HARDWARE_INFO),
	channel_()
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

	if (msg.checkBufferSize<MsgHardwareInfo2>())
	{
		MsgHardwareInfo2 hwInfo = msg.read<MsgHardwareInfo2>();

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

		hardwareInfoMsg_.brainstemSwVersion = hwInfo.brainstemFirmwareVersion;

		uint8_t numberOfBatteries = hwInfo.numberOfBatteries;

		ROS_INFO_STREAM("Number of batteries: " << (int)numberOfBatteries);

		for (int i = 0; i < numberOfBatteries; i++)
		{
			MsgBatteryInfo batteryInfo = msg.read<MsgBatteryInfo>();

			srslib_framework::MsgBatteryInfo batteryInfoMsg;
			batteryInfoMsg.manufacturer = batteryInfo.manufacturer;
			batteryInfoMsg.serialNumber = batteryInfo.serialNumber;

			hardwareInfoMsg_.batteryInfo.push_back(batteryInfoMsg);

			batteryStream << ", battery" << i << ": manufacturer=" << batteryInfoMsg.manufacturer <<
				", serial#=" << batteryInfoMsg.serialNumber;
		}
	}
	else
	{
		HardwareInfoMsg1 hwInfo = msg.read<HardwareInfoMsg1>();

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

	ROS_INFO_STREAM("Hardware Info {" <<
		"name: " << hardwareInfoMsg_.name <<
		", uid: " << hardwareInfoMsg_.uid <<
		", chassis generation: " << (int)hardwareInfoMsg_.chassisGeneration <<
		", brainstem hw version: " << (int)hardwareInfoMsg_.brainstemHwVersion <<
		", brainstem sw version: " << hardwareInfoMsg_.brainstemSwVersion <<
		batteryStream.str() << "}");


	// Publish the hardware info
	channel_.publish(hardwareInfoMsg_);
}

} // namespace srs
