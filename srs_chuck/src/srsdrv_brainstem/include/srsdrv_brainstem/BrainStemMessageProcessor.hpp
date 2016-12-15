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
#include <srslib_framework/ros/channel/ChannelBrainstemHardwareInfo.hpp>
#include <srslib_framework/ros/channel/ChannelBrainstemPowerState.hpp>
#include <srslib_framework/ros/channel/ChannelBrainstemOdometryRpm.hpp>
#include <srslib_framework/ros/channel/ChannelBrainstemButtonPressed.hpp>

#include <BrainStemMessages.hpp>
#include <BrainStemMessageProcessorInterface.hpp>
#include <srsdrv_brainstem/HardwareMessageHandler.hpp>

#include <srsdrv_brainstem/hw_message/LogHandler.hpp>
#include <srsdrv_brainstem/hw_message/HardwareInfoHandler.hpp>
#include <srsdrv_brainstem/hw_message/OperationalStateHandler.hpp>
#include <srsdrv_brainstem/hw_message/PowerStateHandler.hpp>
#include <srsdrv_brainstem/hw_message/OdometryRpmHandler.hpp>
#include <srsdrv_brainstem/hw_message/ButtonPressedHandler.hpp>

#include <srsdrv_brainstem/sw_message/PingHandler.hpp>
#include <srsdrv_brainstem/sw_message/SetMotionStateHandler.hpp>
#include <srsdrv_brainstem/sw_message/SetOdometryRpmHandler.hpp>
#include <srsdrv_brainstem/sw_message/ShutdownHandler.hpp>
#include <srsdrv_brainstem/sw_message/SoundHandler.hpp>
#include <srsdrv_brainstem/sw_message/UpdateUIHandler.hpp>

namespace srs {

class IO;
class MessageProcessor;

class BrainStemMessageProcessor : public BrainStemMessageProcessorInterface
{
public:
    void processHardwareMessage(vector<char> buffer);

    using HwMessageHandlerMapType = map<BRAIN_STEM_MSG, HardwareMessageHandler*>;

public:

	BrainStemMessageProcessor( std::shared_ptr<IO> pIO );

	virtual ~BrainStemMessageProcessor( );

	void checkForBrainstemFaultTimer(const ros::TimerEvent& event);

    void sendCommand(char* command, std::size_t size)
    {
        writeToSerialPort(command, size);
    }

// Message Processing

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

	static constexpr auto FAULT_TIMEOUT = 0.2f;

	std::shared_ptr<IO>					io_;

	bool								isConnected_;

	bool								hasValidHardareInfo_;
	ros::Time							lastHardareInfoRequestTime_;

	bool								hasValidOperationalState_;
	ros::Time							lastOperationalStateRequestTime_;

	ros::Time							lastMessageTime_;

    HwMessageHandlerMapType				hwMessageHandlers_;

	ChannelBrainstemConnected			connectedChannel_;
    ChannelBrainstemHardwareInfo		hardwareInfoChannel_;
    ChannelBrainstemOperationalState	operationalStateChannel_;
    ChannelBrainstemPowerState			powerStateChannel_;
    ChannelBrainstemOdometryRpm			odometryRpmChannel_;
    ChannelBrainstemButtonPressed		buttonPressedChannel_;

    LogHandler							logHandler_;
    HardwareInfoHandler					hardwareInfoHandler_;
    OperationalStateHandler				operationalStateHandler_;
    PowerStateHandler					powerStateHandler_;
    OdometryRpmHandler					odometryRpmHandler_;
    ButtonPressedHandler				buttonPressedHandler_;

    PingHandler							pingHandler_;
    SetMotionStateHandler				setMotionStateHandler_;
    SetOdometryRpmHandler				setOdometryRpmHandler_;
    ShutdownHandler						shutdownHandler_;
    SoundHandler						soundHandler_;
    UpdateUIHandler						updateUIHandler_;
};

} /* namespace srs */
