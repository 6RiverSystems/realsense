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

class MessageProcessor {
private:

	ros::NodeHandle&				m_node;

	IO*								m_pIO;

	ros::Subscriber					m_llcmdSubscriber;

	ros::Publisher					m_llCmdPublisher;

	ros::Publisher					m_llEventPublisher;

	ros::Publisher					m_llSensorsPublisher;

	std::map<ENTITIES, std::string>	m_mapButtons;

public:

	MessageProcessor( ros::NodeHandle& node, IO* pIO );

	virtual ~MessageProcessor( );

	void ProcessMessage( std::vector<char> buffer );

	void OnRosCallback( const std_msgs::String::ConstPtr& msg );

};

} /* namespace srs */

#endif /* MESSAGEPROCESSOR_H_ */
