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

#include <srsdrv_brainstem/BrainStemMessages.h>
#include <srsdrv_brainstem/hw_message/HardwareMessageHandler.hpp>
#include <srsdrv_brainstem/hw_message/SensorFrameHandler.hpp>
#include <srsdrv_brainstem/hw_message/RawOdometryHandler.hpp>
#include <srsdrv_brainstem/hw_message/HardwareInfoHandler.hpp>

#include <srsdrv_brainstem/sw_message/HonkHandler.hpp>

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
    void processRosMessage(const string& strMessage);

    using HwMessageHandlerMapType = map<char, HardwareMessageHandler*>;

	typedef std::function<void(bool)> ConnectionChangedFn;

	typedef std::function<void(LED_ENTITIES)> ButtonCallbackFn;

	typedef std::function<void(uint32_t, float, float)> OdometryCallbackFn;

	typedef std::function<void(uint32_t[4], uint8_t,
		uint8_t, const std::string&)> HardwareInfoCallbackFn;

	typedef std::function<void(uint32_t, const MOTION_STATUS_DATA&,
		const FAILURE_STATUS_DATA&)> OperationalStateCallbackFn;

	typedef std::function<void(float)> VoltageCallbackFn;

private:

	std::shared_ptr<IO>						m_pIO;

	std::map<LED_ENTITIES, std::string>		m_mapEntityButton;

	std::map<std::string, LED_ENTITIES>		m_mapButtonEntity;

	std::map<std::string, LED_MODE>			m_mapLedMode;

	std::map<std::string, MOTION_STATUS>	m_mapMotionStatus;

	std::map<std::string, Handler>			m_vecBridgeCallbacks;

	ConnectionChangedFn						m_connectionChangedCallback;

	ButtonCallbackFn						m_buttonCallback;

	OperationalStateCallbackFn				m_operationalStateCallback;

	VoltageCallbackFn						m_voltageCallback;

public:

	BrainStemMessageProcessor( std::shared_ptr<IO> pIO );

	virtual ~BrainStemMessageProcessor( );

    void sendCommand(char* command, std::size_t size)
    {
        WriteToSerialPort(command, size);
    }

    // Message Callbacks

	void SetConnectionChangedCallback( ConnectionChangedFn connectionChangedCallback );

	void SetButtonCallback( ButtonCallbackFn buttonCallback );

	void SetOperationalStateCallback( OperationalStateCallbackFn operationalStateCallback );

	void SetVoltageCallback( VoltageCallbackFn voltageCallback );

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

	void OnPause( std::vector<std::string> vecParams );

	void ClearMotionStatus( );

	void OnStartup( std::vector<std::string> vecParams );

// Helper Methods

	void WriteToSerialPort( char* pszData, std::size_t dwSize );

private:

	void Pause( bool bPause );

    HwMessageHandlerMapType hwMessageHandlers_;

    HardwareInfoHandler hardwareInfoHandler_;
    HonkHandler honkHandler_;

    SensorFrameHandler sensorFrameHandler_;

    RawOdometryHandler rawOdometryHandler_;
};

} /* namespace srs */

#endif /* BRAINSTEM_MBRAINSTEM_MESSAGEPROCESSOR_H_ESSAGEPROCESSOR_H_ */
