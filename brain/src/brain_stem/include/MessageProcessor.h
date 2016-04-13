/*
 * MessageProcessor.h
 *
 *  Created on: Apr 7, 2016
 *      Author: dan
 */

#include <vector>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include "Messages.h"

#ifndef MESSAGEPROCESSOR_H_
#define MESSAGEPROCESSOR_H_

namespace srs {

class IO;
class MessageProcessor;

struct Handler
{
	std::function<void(std::vector<std::string>)>	callback;

	uint32_t										dwNumParams;

};

class MessageProcessor {
private:

	bool							m_bControllerFault;

	ros::NodeHandle&				m_node;

	IO*								m_pIO;

	ros::Subscriber					m_llcmdSubscriber;

	ros::Publisher					m_llEventPublisher;

	ros::Publisher					m_llSensorsPublisher;

	std::map<ENTITIES, std::string>	m_mapEntityButton;

	std::map<std::string, ENTITIES>	m_mapButtonEntity;

	std::map<std::string, LED_MODE>	m_mapLedMode;

	std::map<std::string, Handler>	m_vecBridgeCallbacks;

public:

	MessageProcessor( ros::NodeHandle& node, IO* pIO );

	virtual ~MessageProcessor( );

	void ProcessMessage( std::vector<char> buffer );

	void OnRosCallback( const std_msgs::String::ConstPtr& msg );

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

	void OnSetPid( std::vector<std::string> vecParams );

// Helper Methods

	void WriteToSerialPort( char* pszData, std::size_t dwSize );

};

} /* namespace srs */

#endif /* MESSAGEPROCESSOR_H_ */