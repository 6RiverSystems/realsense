/*
 * MessageProcessor.cpp
 *
 *  Created on: Apr 7, 2016
 *      Author: dan
 */

#include "MessageProcessor.h"
#include "Messages.h"

#include <chrono>

#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/String.h>

namespace srs {

using namespace ros;

MessageProcessor::MessageProcessor( ros::NodeHandle& node, IO* pIO ) :
	m_node( node ),
	m_pIO( pIO ),
	m_llcmdSubscriber( node.subscribe<std_msgs::String>( "/cmd_ll", 1000,
		std::bind( &MessageProcessor::OnRosCallback, this, std::placeholders::_1 ) ) ),
	m_llCmdPublisher( node.advertise<std_msgs::String>( "/cmd_ll", 50 ) ),
	m_llEventPublisher( node.advertise<std_msgs::String>( "/ll_event", 50 ) ),
	m_llSensorsPublisher( node.advertise<std_msgs::String>( "/ll_sensors", 1000 ) )
{
	m_mapButtons[ENTITIES::LIGHT_TOTE0]			= "TOTE0";
	m_mapButtons[ENTITIES::LIGHT_TOTE1]			= "TOTE1";
	m_mapButtons[ENTITIES::LIGHT_TOTE2]			= "TOTE2";
	m_mapButtons[ENTITIES::LIGHT_TOTE3]			= "TOTE3";
	m_mapButtons[ENTITIES::LIGHT_TOTE4]			= "TOTE4";
	m_mapButtons[ENTITIES::LIGHT_TOTE5]			= "TOTE5";
	m_mapButtons[ENTITIES::LIGHT_TOTE6]			= "TOTE6";
	m_mapButtons[ENTITIES::LIGHT_TOTE7]			= "TOTE7";
	m_mapButtons[ENTITIES::LIGHT_ACTION]		= "ACTION";
	m_mapButtons[ENTITIES::LIGHT_PAUSE]			= "PAUSE";
	m_mapButtons[ENTITIES::LIGHT_TAIL_LEFT]		= "TAIL_LEFT";
	m_mapButtons[ENTITIES::LIGHT_TAIL_RIGHT]	= "TAIL_RIGHT";
}

MessageProcessor::~MessageProcessor( )
{

}


void MessageProcessor::ProcessMessage( std::vector<char> buffer )
{
	BRAIN_STEM_MESSAGE eCommand;

	if( buffer.size( ) > 0 )
	{
		eCommand = static_cast<BRAIN_STEM_MESSAGE>( buffer[0] );
	}

	switch( eCommand )
	{
		case BRAIN_STEM_MESSAGE::MESSAGE:
		{
			std::string strMessage( buffer.begin( ), buffer.end( ) );

			if( strMessage.find( "<MSG Error" ) != -1 )
			{
				ROS_ERROR( "Fatal Error: %s", strMessage.c_str( ) );

				// TODO: Send a fault up to the brainstem
			}
			else
			{
				ROS_DEBUG( "Message from BrainStem: %s", strMessage.c_str( ) );
			}
		}
		break;

		case BRAIN_STEM_MESSAGE::STOP:
		{
			std_msgs::String message;

			message.data = "ARRIVED";

			m_llCmdPublisher.publish( message );
		}
		break;

		case BRAIN_STEM_MESSAGE::BUTTON:
		{
			ENTITIES eButtonId = static_cast<ENTITIES>( buffer[1] );

			std::string strEntity;

			auto iter = m_mapButtons.find( eButtonId );

			if( iter != m_mapButtons.end( ) )
			{
				std_msgs::String msg;

				std::stringstream ss;
				ss << "UI " << iter->second;
				msg.data = ss.str();

				m_llEventPublisher.publish( msg );
			}
			else
			{
				ROS_ERROR( "Unknown button entity %d", eButtonId );
			}
		}
		break;

		case BRAIN_STEM_MESSAGE::ODOMETRY:
		{
			ODOMETRY_DATA* pOdometry = reinterpret_cast<ODOMETRY_DATA*>( buffer.data( ) + 1 );

			std_msgs::String msg;

            double fractional_seconds_since_epoch = std::chrono::duration_cast<std::chrono::duration<double>>(
            	std::chrono::system_clock::now( ).time_since_epoch( ) ).count( );

			std::stringstream ss;
			ss << "O " << pOdometry->left_encoder << "," << pOdometry->right_encoder << " @ " << fractional_seconds_since_epoch;
			msg.data = ss.str();

			m_llEventPublisher.publish( msg );
		}
		break;

		case BRAIN_STEM_MESSAGE::PID:
		{
			// TODO: Define real pid messages to help with tuning
			PID_DATA* pPid = reinterpret_cast<PID_DATA*>( buffer.data( ) + 1 );
			// pubXEst.publish("X_DESIRED %f" % pidData[1])
			// pubXEst.publish("X_CURRENT %f" % pidData[1])
		}
		break;

		default:
		{
			ROS_DEBUG( "Unknown MFP command: %d", eCommand );
		}
		break;
	}
}

void MessageProcessor::OnRosCallback( const std_msgs::String::ConstPtr& msg )
{

}


} /* namespace srs */
