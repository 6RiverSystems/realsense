/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <vector>
#include <map>
#include <functional>
#include <memory>

using namespace std;

#include <srslib_framework/io/IO.hpp>
#include <srslib_framework/ros/channel/ChannelBrainstemConnected.hpp>
#include <srslib_framework/ros/channel/ChannelBrainstemConnected.hpp>
#include <srslib_framework/ros/channel/ChannelBrainstemHardwareInfo.hpp>
#include <srslib_framework/ros/channel/ChannelBrainstemPowerState.hpp>
#include <srslib_framework/ros/channel/ChannelBrainstemOdometryRpm.hpp>
#include <srslib_framework/ros/channel/ChannelBrainstemOdometryPose.hpp>
#include <srslib_framework/ros/channel/ChannelBrainstemButtonPressed.hpp>

#include <BrainStemMessages.hpp>
#include <BrainStemMessageProcessorInterface.hpp>

#include <hw_message/HardwareMessageHandler.hpp>

#include <hw_message/LogHandler.hpp>
#include <hw_message/HardwareInfoHandler.hpp>
#include <hw_message/OperationalStateHandler.hpp>
#include <hw_message/PowerStateHandler.hpp>
#include <hw_message/OdometryRpmHandler.hpp>
#include <hw_message/OdometryPoseHandler.hpp>
#include <hw_message/ButtonPressedHandler.hpp>

#include <sw_message/ResetHandler.hpp>
#include <sw_message/PingHandler.hpp>
#include <sw_message/SetMotionStateHandler.hpp>
#include <sw_message/SetOdometryRpmHandler.hpp>
#include <sw_message/SetVelocityHandler.hpp>
#include <sw_message/ShutdownHandler.hpp>
#include <sw_message/SoundHandler.hpp>
#include <sw_message/UpdateUIHandler.hpp>

namespace srs {

class IO;
class MessageProcessor;

class BrainStemMessageProcessor : public BrainStemMessageProcessorInterface
{
public:

	typedef std::shared_ptr<vector<char>> MessageBuffer;

	enum class DIMENSION
	{
		WHEEL_BASE_LENGTH = 0,
		LEFT_WHEEL_RADIUS = 1,
		RIGHT_WHEEL_RADIUS = 2
	};

	void processHardwareMessage(vector<char> buffer);

    using HwMessageHandlerMapType = map<BRAIN_STEM_MSG, HardwareMessageHandler*>;

	BrainStemMessageProcessor( std::shared_ptr<IO> pIO );

	virtual ~BrainStemMessageProcessor( );

	void checkForBrainstemFaultTimer(const ros::TimerEvent& event);

	bool isConnected() const
	{
		return isConnected_;
	}

    void sendCommand(char* command, std::size_t size, bool resync = false);

    void ping();

// Message Processing

    void setDimension(DIMENSION dimension, float value);

	void setConnected( bool isConnected );

	void getOperationalState( );

	void getHardwareInformation( );

    void setMotionStatus( const std::bitset<8>& motionStatusSet, bool setValue );

    void shutdown( );

private:

	bool isSetupComplete() const;

	void checkSetupComplete();

	void getHardwareInfo(const ros::Time& now);

	void getOperationalState(const ros::Time& now);

	void sendDimensions();

// Helper Methods

	void writeToSerialPort( char* data, std::size_t size );

private:

    HW_MESSAGE_BEGIN(CommandData)
        uint8_t cmd;
    HW_MESSAGE_END

    HW_MESSAGE_BEGIN(OperationalStateData)
    	uint8_t cmd;
		uint8_t motionStatus;
    HW_MESSAGE_END

    HW_MESSAGE_BEGIN(DimensionData)
    	uint8_t cmd;
		uint8_t id;
		float value;
    HW_MESSAGE_END

	static constexpr auto FAULT_TIMEOUT = 0.2f;

	std::shared_ptr<IO> io_;

	bool setupComplete_;

	bool syncState_;

	bool sentPing_;

	bool isConnected_;

	std::map<BRAIN_STEM_CMD,MessageBuffer> syncMessages_;

	std::map<DIMENSION, float> dimensions_;

	bool hasValidHardareInfo_;
	ros::Time lastHardareInfoRequestTime_;

	bool hasValidOperationalState_;
	ros::Time lastOperationalStateRequestTime_;

	ros::Time lastMessageTime_;

    HwMessageHandlerMapType hwMessageHandlers_;

	ChannelBrainstemConnected connectedChannel_;
    ChannelBrainstemHardwareInfo hardwareInfoChannel_;
    ChannelBrainstemOperationalState operationalStateChannel_;
    ChannelBrainstemPowerState powerStateChannel_;
    ChannelBrainstemOdometryRpm odometryRpmChannel_;
    ChannelBrainstemOdometryPose odometryPoseChannel_;
    ChannelBrainstemButtonPressed buttonPressedChannel_;

    LogHandler logHandler_;
    HardwareInfoHandler hardwareInfoHandler_;
    OperationalStateHandler operationalStateHandler_;
    PowerStateHandler powerStateHandler_;
    OdometryRpmHandler odometryRpmHandler_;
    OdometryPoseHandler odometryPoseHandler_;
    ButtonPressedHandler buttonPressedHandler_;

    ResetHandler resetHandler_;
    PingHandler pingHandler_;
    SetMotionStateHandler setMotionStateHandler_;
    SetOdometryRpmHandler setOdometryRpmHandler_;
    SetVelocityHandler setVelocityHandler_;
    ShutdownHandler shutdownHandler_;
    SoundHandler soundHandler_;
    UpdateUIHandler updateUIHandler_;
};

} /* namespace srs */
