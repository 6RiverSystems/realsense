/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */

#ifndef BRAINSTEM_MESSAGEPROCESSOR_H_
#define BRAINSTEM_MESSAGEPROCESSOR_H_

#include <vector>
#include <map>
#include <functional>
#include <memory>
#include <bitset>

using namespace std;

#include <srslib_framework/io/IO.hpp>
#include <srslib_framework/ros/channel/ChannelBrainstemHardwareInfo.hpp>
#include <srslib_framework/ros/channel/ChannelBrainstemPowerState.hpp>
#include <srslib_framework/ros/channel/ChannelBrainstemSensorFrame.hpp>
#include <srslib_framework/ros/channel/ChannelBrainstemOdometryRpm.hpp>
#include <srslib_framework/ros/channel/ChannelBrainstemButtonPressed.hpp>

#include <srsdrv_brainstem/BrainStemMessages.h>

#include <srsdrv_brainstem/hw_message/HardwareMessageHandler.hpp>
#include <srsdrv_brainstem/hw_message/SensorFrameHandler.hpp>
#include <srsdrv_brainstem/hw_message/HardwareInfoHandler.hpp>
#include <srsdrv_brainstem/hw_message/PowerStateHandler.hpp>
#include <srsdrv_brainstem/hw_message/OdometryRpmHandler.hpp>
#include <srsdrv_brainstem/hw_message/ButtonPressedHandler.hpp>

#include <srsdrv_brainstem/sw_message/SoundHandler.hpp>
#include <srsdrv_brainstem/sw_message/FreeSpinHandler.hpp>
#include <srsdrv_brainstem/sw_message/SetMotionStateHandler.hpp>
#include <srsdrv_brainstem/sw_message/ClearMotionStateHandler.hpp>
#include <srsdrv_brainstem/sw_message/PingHandler.hpp>

namespace srs {

class IO;
class MessageProcessor;

struct Handler
{
	std::function<void(std::vector<std::string>)>	callback;

	uint32_t										dwNumParams;
};

class BrainStemMessageProcessor
{
public:
    void processHardwareMessage(vector<char> buffer);

    using HwMessageHandlerMapType = map<BRAIN_STEM_MSG, HardwareMessageHandler*>;

public:

	BrainStemMessageProcessor( std::shared_ptr<IO> pIO );

	virtual ~BrainStemMessageProcessor( );

    void sendCommand(char* command, std::size_t size)
    {
        WriteToSerialPort(command, size);
    }

    void setMotionStatus(const std::bitset<8>& motionStatusSet, bool bSetValues)
    {
        SetMotionStatus(motionStatusSet, bSetValues);
    }

// Message Processing

	void GetOperationalState( );

	void GetHardwareInformation( );

	void SendPing( );

	void SetRPM( double leftWheelRPM, double rightWheelRPM );

	void SetVelocity( double dfLinear, double dfAngular );

	void SetConnected( bool bIsConnected );

    void SetMotionStatus( const std::bitset<8>& motionStatusSet, bool bSetValues );

    void OnHardStop( );

    void Shutdown( );

// Helper

	std::string GetButtonName( LED_ENTITIES eButtonId ) const;

private:

// Bridge Callbacks

	void OnResetBatteryHours( );

	void OnResetWheelMeters( );

	void OnSetConfiguration( uint32_t configuration );

	void OnUpdateLights( std::vector<std::string> vecParams );

	void ClearMotionStatus( );

	void OnStartup( std::vector<std::string> vecParams );

// Helper Methods

	void WriteToSerialPort( char* pszData, std::size_t dwSize );

private:

	std::shared_ptr<IO>				m_pIO;

    HwMessageHandlerMapType			hwMessageHandlers_;

    ChannelBrainstemHardwareInfo	hardwareInfoChannel_;
    ChannelBrainstemPowerState		powerStateChannel_;
    ChannelBrainstemSensorFrame		sensorFrameChannel_;
    ChannelBrainstemOdometryRpm		odometryRpmChannel_;
    ChannelBrainstemButtonPressed	buttonPressedChannel_;

    HardwareInfoHandler				hardwareInfoHandler_;
    PowerStateHandler				powerStateHandler_;
    SensorFrameHandler				sensorFrameHandler_;
    OdometryRpmHandler				odometryRpmHandler_;
    ButtonPressedHandler			buttonPressedHandler_;

    SoundHandler					soundHandler_;
    FreeSpinHandler					freeSpinHandler_;
    SetMotionStateHandler			setMotionStateHandler_;
    ClearMotionStateHandler			clearMotionStateHandler_;
    PingHandler						pingHandler_;
};

} /* namespace srs */

#endif /* BRAINSTEM_MBRAINSTEM_MESSAGEPROCESSOR_H_ESSAGEPROCESSOR_H_ */
