/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */

#include <vector>
#include <map>
#include <functional>
#include <memory>
#include <BrainStemMessages.h>
#include <srslib_framework/io/IO.hpp>

#ifndef BRAINSTEM_MESSAGEPROCESSOR_H_
#define BRAINSTEM_MESSAGEPROCESSOR_H_

namespace srs {

class IO;
class MessageProcessor;

struct Handler
{
	std::function<void(std::vector<std::string>)>	callback;

	uint32_t										dwNumParams;
};

class BrainStemMessageProcessor {

	typedef std::function<void(bool)> ConnectionChangedFn;

	typedef std::function<void(LED_ENTITIES)> ButtonCallbackFn;

	typedef std::function<void(uint32_t, float, float)> OdometryCallbackFn;

	typedef std::function<void(uint32_t[4], uint8_t,
		uint32_t, uint32_t, uint32_t, uint32_t,
		uint32_t, std::string)> HardwareInfoCallbackFn;

	typedef std::function<void(uint32_t, MOTION_STATUS_DATA,
		FAILURE_STATUS_DATA, uint8_t)> OperationalStateCallbackFn;

	typedef std::function<void(float)> VoltageCallbackFn;

private:

	std::shared_ptr<IO>				m_pIO;

	std::map<LED_ENTITIES, std::string>	m_mapEntityButton;

	std::map<std::string, LED_ENTITIES>	m_mapButtonEntity;

	std::map<std::string, LED_MODE>	m_mapLedMode;

	std::map<std::string, Handler>	m_vecBridgeCallbacks;

	ConnectionChangedFn				m_connectionChangedCallback;

	ButtonCallbackFn				m_buttonCallback;

	OdometryCallbackFn				m_odometryCallback;

	HardwareInfoCallbackFn			m_hardwareInfoCallback;

	OperationalStateCallbackFn		m_operationalStateCallback;

	VoltageCallbackFn				m_voltageCallback;

public:

	BrainStemMessageProcessor( std::shared_ptr<IO> pIO );

	virtual ~BrainStemMessageProcessor( );

// Message Callbacks

	void SetConnectionChangedCallback( ConnectionChangedFn connectionChangedCallback );

	void SetButtonCallback( ButtonCallbackFn buttonCallback );

	void SetOdometryCallback( OdometryCallbackFn odometryCallback );

	void SetHardwareInfoCallback( HardwareInfoCallbackFn hardwareInfoCallback );

	void SetOperationalStateCallback( OperationalStateCallbackFn operationalStateCallback );

	void SetVoltageCallback( VoltageCallbackFn voltageCallback );

// Message Processing

	void ProcessBrainStemMessage( std::vector<char> buffer );

	void ProcessRosMessage( const std::string& strMessage );

	void GetOperationalState( );

	void GetHardwareInformation( );

	void SendPing( );

	void SetVelocity( double dfLinear, double dfAngular );

	void SetConnected( bool bIsConnected );

	void Shutdown( );

// Helper

	std::string GetButtonName( LED_ENTITIES eButtonId ) const;

private:

// Bridge Callbacks

	void SetMotionStatus( MOTION_STATUS eMotionStatus, bool bSetBit );

	void OnHardStop( );

	void OnResetBatteryHours( );

	void OnResetWheelMeters( );

	void OnSetConfiguration( uint32_t configuration );

	void OnUpdateLights( std::vector<std::string> vecParams );

	void OnPause( std::vector<std::string> vecParams );

	void OnStartup( std::vector<std::string> vecParams );

// Helper Methods

	void WriteToSerialPort( char* pszData, std::size_t dwSize );

private:

	void Pause( bool bPause );

};

} /* namespace srs */

#endif /* BRAINSTEM_MBRAINSTEM_MESSAGEPROCESSOR_H_ESSAGEPROCESSOR_H_ */
