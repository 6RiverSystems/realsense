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
#include <srslib_framework/ros/channel/ChannelBrainstemPowerStateFiltered.hpp>
#include <srslib_framework/ros/channel/ChannelBrainstemOdometryRpm.hpp>
#include <srslib_framework/ros/channel/ChannelBrainstemOdometryPose.hpp>
#include <srslib_framework/ros/channel/ChannelBrainstemOdometryPoseBrainstem.hpp>
#include <srslib_framework/ros/channel/ChannelBrainstemButtonPressed.hpp>

#include <BrainStemMessages.hpp>
#include <BrainStemMessageProcessorInterface.hpp>

#include <filters/PowerStateFilter.hpp>

#include <hw_message/HardwareMessageHandler.hpp>
#include <sw_message/SoftwareMessage.hpp>

#include <hw_message/HardwareInfoHandler.hpp>
#include <hw_message/OperationalStateHandler.hpp>
#include <hw_message/OdometryPoseHandler.hpp>

#include <command/SetPhysicalDimension.hpp>

namespace srs {

class IO;
class MessageProcessor;

class BrainStemMessageProcessor : public BrainStemMessageProcessorInterface
{
public:
	void processHardwareMessage(vector<char> buffer);

    using HwMessageHandlerMapType = map<BRAIN_STEM_MSG, HardwareMessageHandlerPtr>;

	BrainStemMessageProcessor();

	virtual ~BrainStemMessageProcessor();

	void setIO(std::shared_ptr<IO> io) { io_ = io; };

	void checkForBrainstemFaultTimer(const ros::TimerEvent& event);

	bool isConnected() const
	{
		return isConnected_;
	}

    void sendCommand(char* command, std::size_t size);

    void ping();

// Message Processing

    void setUseBrainstemOdom(bool useBrainstemOdom);

    void setDimension(SetPhysicalDimension::DimensionEnum dimension, float value);

	void setConnected( bool isConnected );

	void getOperationalState( );

	void getHardwareInformation( );

private:
    void addHardwareMessageHandler(HardwareMessageHandlerPtr hardwareMessageHandler);

    void removeHardwareMessageHandler(HardwareMessageHandlerPtr hardwareMessageHandler);

    void addSoftwareMessage(SoftwareMessagePtr softwareMessage);

    void removeSoftwareMessage(SoftwareMessagePtr softwareMessage);

	bool isSetupComplete() const;

	void checkSetupComplete();

	void getHardwareInfo(const ros::Time& now);

	void getOperationalState(const ros::Time& now);

	void sendDimensions();
	void sendMaxAllowedVelocity();

// Helper Methods

	void writeToSerialPort( char* data, std::size_t size );

private:

	static constexpr auto FAULT_TIMEOUT = 0.2f;

    std::shared_ptr<IO> io_;

	bool setupComplete_;

	bool syncState_;

	bool sentPing_;

	bool isConnected_;

    std::map<SetPhysicalDimension::DimensionEnum, float> dimensions_;

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
    ChannelBrainstemPowerStateFiltered powerStateFilteredChannel_;
    ChannelBrainstemOdometryRpm odometryRpmChannel_;
    ChannelBrainstemOdometryPoseBrainstem odometryPoseBrainstemChannel_;
    ChannelBrainstemOdometryPose odometryPoseChannel_;
    ChannelBrainstemButtonPressed buttonPressedChannel_;

    PowerStateFilter powerStateFilter_;

    std::shared_ptr<HardwareInfoHandler> hardwareInfoHandler_;
    std::shared_ptr<OperationalStateHandler> operationalStateHandler_;
    std::shared_ptr<OdometryPoseHandler> odometryPoseHardwarewHandler_;

    bool useBrainstemOdom_;

    HardwareMessageHandlerPtr odometryRpmHardwareHandler_;
    //HardwareMessageHandlerPtr odometryPoseHardwarewHandler_;

    SoftwareMessagePtr velocitySoftwareHandler_;
    SoftwareMessagePtr odometryRpmSoftwareHandler_;

    std::set<HardwareMessageHandlerPtr> hardwareHandlers_;
    std::set<SoftwareMessagePtr> softwareHandlers_;
};

} /* namespace srs */
