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

	typedef std::function<void()> ArrivedCallbackFn;

	typedef std::function<void(ENTITIES)> ButtonCallbackFn;

	typedef std::function<void(uint32_t, float, float)> OdometryCallbackFn;

private:

	bool							m_bControllerFault;

	std::shared_ptr<IO>				m_pIO;

	std::map<ENTITIES, std::string>	m_mapEntityButton;

	std::map<std::string, ENTITIES>	m_mapButtonEntity;

	std::map<std::string, LED_MODE>	m_mapLedMode;

	std::map<std::string, Handler>	m_vecBridgeCallbacks;

	ConnectionChangedFn				m_connectionChangedCallback;

	ArrivedCallbackFn				m_arrivedCallback;

	ButtonCallbackFn				m_buttonCallback;

	OdometryCallbackFn				m_odometryCallback;

public:

	BrainStemMessageProcessor( std::shared_ptr<IO> pIO );

	virtual ~BrainStemMessageProcessor( );

// Message Callbacks

	void SetConnectionChangedCallback( ConnectionChangedFn connectionChangedCallback );

	void SetArrivedCallback( ArrivedCallbackFn arrivedCallback );

	void SetButtonCallback( ButtonCallbackFn buttonCallback );

	void SetOdometryCallback( OdometryCallbackFn odometryCallback );

// Message Processing

	void ProcessBrainStemMessage( std::vector<char> buffer );

	void ProcessRosMessage( const std::string& strMessage );

	void SetVelocity( double dfLinear, double dfAngular );

	void SetConnected( bool bIsConnected );

// Helper

	std::string GetButtonName( ENTITIES eButtonId ) const;

private:

// Bridge Callbacks

	void OnUI( std::vector<std::string> vecParams );

	void OnStartup( std::vector<std::string> vecParams );

	void OnDistance( std::vector<std::string> vecParams );

	void OnRotate( std::vector<std::string> vecParams );

	void OnStop( std::vector<std::string> vecParams );

	void OnTurn( std::vector<std::string> vecParams );

	void OnVersion( std::vector<std::string> vecParams );

	void OnPause( std::vector<std::string> vecParams );

	void OnReEnable( std::vector<std::string> vecParams );

// Helper Methods

	void WriteToSerialPort( char* pszData, std::size_t dwSize );

};

} /* namespace srs */

#endif /* BRAINSTEM_MBRAINSTEM_MESSAGEPROCESSOR_H_ESSAGEPROCESSOR_H_ */