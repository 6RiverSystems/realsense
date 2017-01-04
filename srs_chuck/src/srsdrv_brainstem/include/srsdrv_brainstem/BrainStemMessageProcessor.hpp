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
#include <sw_message/SoftwareMessage.hpp>

#include <hw_message/HardwareInfoHandler.hpp>
#include <hw_message/OperationalStateHandler.hpp>

namespace srs {

class IO;
class MessageProcessor;

class BrainStemMessageProcessor : public BrainStemMessageProcessorInterface
{
public:

	enum class DIMENSION
	{
		WHEEL_BASE_LENGTH = 0,
		LEFT_WHEEL_RADIUS = 1,
		RIGHT_WHEEL_RADIUS = 2
	};

	void processHardwareMessage(vector<char> buffer);

    using HwMessageHandlerMapType = map<BRAIN_STEM_MSG, HardwareMessageHandlerPtr>;

	BrainStemMessageProcessor( std::shared_ptr<IO> pIO );

	virtual ~BrainStemMessageProcessor( );

	void checkForBrainstemFaultTimer(const ros::TimerEvent& event);

	bool isConnected() const
	{
		return isConnected_;
	}

    void sendCommand(char* command, std::size_t size);

    void ping();

// Message Processing

    void setDimension(DIMENSION dimension, float value);

	void setConnected( bool isConnected );

	void getOperationalState( );

	void getHardwareInformation( );

    void shutdown( );

private:

    void addHardwareMessageHandler(HardwareMessageHandlerPtr hardwareMessageHandler);

    void addSoftwareMessage(SoftwareMessagePtr softwareMessage);

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

	std::map<DIMENSION, float> dimensions_;

	bool hasValidHardareInfo_;
	ros::Time lastHardareInfoRequestTime_;

	bool hasValidOperationalState_;
	ros::Time lastOperationalStateRequestTime_;

	ros::Time lastMessageTime_;

    HwMessageHandlerMapType	hwMessageHandlers_;

	ChannelBrainstemConnected	connectedChannel_;
    ChannelBrainstemHardwareInfo	hardwareInfoChannel_;
    ChannelBrainstemOperationalState	operationalStateChannel_;
    ChannelBrainstemPowerState	powerStateChannel_;
    ChannelBrainstemOdometryRpm	odometryRpmChannel_;
    ChannelBrainstemOdometryPose	odometryPoseChannel_;
    ChannelBrainstemButtonPressed	buttonPressedChannel_;

    std::shared_ptr<HardwareInfoHandler>	hardwareInfoHandler_;
    std::shared_ptr<OperationalStateHandler>	operationalStateHandler_;

    std::vector<HardwareMessageHandlerPtr> hardwareHandlers_;
    std::vector<SoftwareMessagePtr> softwareHandlers_;
};

} /* namespace srs */
