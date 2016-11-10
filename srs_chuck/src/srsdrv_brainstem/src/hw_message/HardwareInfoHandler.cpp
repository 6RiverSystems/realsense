#include <srsdrv_brainstem/hw_message/HardwareInfoHandler.hpp>

#include <srslib_framework/MsgHardwareInfo.h>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Constant definitions

const string HardwareInfoHandler::TOPIC_HARDWARE_INFO = "/info/hardware";

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

HardwareInfoHandler::HardwareInfoHandler() :
    BrainstemMessageHandler(HARDWARE_INFO_KEY),
    chassisGeneration_(0),
    brainstemHwVersion_(0)
{
    pubHardwareInfo_ = rosNodeHandle_.advertise<srslib_framework::MsgHardwareInfo>(
        TOPIC_HARDWARE_INFO, 1, true);

    robotName_ = "";
    char* robotName = getenv("ROBOT_NAME");
    if (robotName)
    {
        robotName_ = string(robotName);
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void HardwareInfoHandler::receiveData(ros::Time currentTime, vector<char>& buffer)
{
    HardwareInfoData* hwInfo = reinterpret_cast<HardwareInfoData*>(buffer.data());

    char rawUid[255];
    sprintf(rawUid, "%04X%04X-%04X-%04X-%04X-%04X%04X%04X",
        hwInfo->uniqueId[0], hwInfo->uniqueId[1],
        hwInfo->uniqueId[2],
        hwInfo->uniqueId[3],
        hwInfo->uniqueId[4],
        hwInfo->uniqueId[5], hwInfo->uniqueId[6], hwInfo->uniqueId[7]);
    uid_ = string(rawUid);

    chassisGeneration_ = static_cast<unsigned int>(hwInfo->chassisGeneration);
    brainstemHwVersion_ = static_cast<unsigned int>(hwInfo->brainstemHwVersion);

    char* brainstemVersionPointer = reinterpret_cast<char*>(buffer.data()) + sizeof(HardwareInfoData);
    brainstemSwVersion_ = string(brainstemVersionPointer);

    ROS_INFO_STREAM("Hardware Info {" <<
        "name: " << robotName_ <<
        ", uid: " << uid_ <<
        ", chassis generation: " << chassisGeneration_ <<
        ", brainstem hw version: " << brainstemHwVersion_ <<
        ", brainstem sw version: " << brainstemSwVersion_ << "}");

    // Publish the hardware info
    publishHardwareInfo();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods

////////////////////////////////////////////////////////////////////////////////////////////////////
void HardwareInfoHandler::publishHardwareInfo()
{
    srslib_framework::MsgHardwareInfo message;
    message.name = robotName_;
    message.uid = uid_;
    message.chassisGeneration = chassisGeneration_;
    message.brainstemHwVersion = brainstemHwVersion_;
    message.brainstemSwVersion = brainstemSwVersion_;

    pubHardwareInfo_.publish(message);
}

} // namespace srs
