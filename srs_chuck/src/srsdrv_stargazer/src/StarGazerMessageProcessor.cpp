/*
 * StarGazerMessageProcessor.cpp
 *
 *  Created on: Apr 27, 2016
 *      Author: cacioppo
 */

#include <ros/ros.h>
#include <chrono>
#include <sstream>
#include <string>
#include <iostream>
#include <boost/tokenizer.hpp>

#include "StarGazerMessage.h"
#include "StarGazerMessageProcessor.h"
#include "StarGazerSerialIO.h"

namespace srs {

StarGazerMessageProcessor::StarGazerMessageProcessor(const char *comPort,
	std::function<void(std::string msg, std::string param)> readCallback,
	std::function<void(int tagID, float x, float y, float z, float angle)> odometryCallback) :
	m_readCallback(readCallback),
	m_odometryCallback(odometryCallback),
	m_bControllerFault(false),
	m_pIO(new StarGazerSerialIO()),
	m_lastTxMessage(),
	m_txMessageQueue(),
	m_lastAck(""),
	m_lastTxTime(std::chrono::milliseconds(0)),
	m_odometryRegex("~\\^I([0-9]*)\\|([+-][0-9]*\\.[0-9]*)\\|([+-][0-9]*\\.[0-9]*)\\|([+-][0-9]*\\.[0-9]*)\\|(-?[0-9]*\\.[0-9]*)"),
	m_messageRegex("~\\$([^\\|]*)(?:\\|([^\\`]*))?"),
	m_highrezclk()
{
	m_pIO->Open(comPort, std::bind(&StarGazerMessageProcessor::RxMsgCallback, this, std::placeholders::_1));
}

StarGazerMessageProcessor::~StarGazerMessageProcessor()
{
	m_pIO->Close();
}

//////////////////////////////////////////////////////////////////////////
// StarGazer Commands
//////////////////////////////////////////////////////////////////////////
void StarGazerMessageProcessor::SendRawCommand(std::string fullCmd)
{
	std::cout << "Rawsend: " << fullCmd << std::endl;
	m_lastTxTime = m_highrezclk.now();

	auto typeStart = fullCmd.begin() + 2;
	auto typeEndField = std::find(typeStart, fullCmd.end(), STARGAZER_SEPERATOR);
	auto typeEndMsg = std::find(typeStart, fullCmd.end(), STARGAZER_RTX);

	// The command ends at the first instance of message end or a seperator
	if (typeEndField < typeEndMsg)
	{
		typeEndMsg = typeEndField;
	}


	m_lastTxMessage = std::string(typeStart, typeEndMsg);

	std::vector<char> cmdVec(fullCmd.begin(), fullCmd.end());
	m_pIO->Write(cmdVec);

}

void StarGazerMessageProcessor::BaseCommand(STAR_GAZER_MESSAGE_TYPES type, std::string cmd) {
//	m_lastTxMessage = cmd;

	// Build the command
	std::string fullCmd("");
	fullCmd += STARGAZER_STX;
	fullCmd += (char)type;
	fullCmd += cmd;
	fullCmd += STARGAZER_RTX;

	m_txMessageQueue.push(fullCmd);

	// If we are the only thing in the queue, that means that
	// there is no outstanding outgoing message.  We can just
	// send out the message
	if (m_txMessageQueue.size() == 1) {
		SendRawCommand(fullCmd);
	}

	std::cout << fullCmd << std::endl;
}


void StarGazerMessageProcessor::BaseWriteCommand(std::string cmd) {
	BaseCommand(STAR_GAZER_MESSAGE_TYPES::WRITE, cmd);
}

void StarGazerMessageProcessor::BaseWriteCommand(std::string cmd, std::string arg1) {
	std::string compoundCommand(cmd);
	compoundCommand += STARGAZER_SEPERATOR;
	compoundCommand += arg1;
	BaseCommand(STAR_GAZER_MESSAGE_TYPES::WRITE, compoundCommand);
}

void StarGazerMessageProcessor::BaseWriteCommand(std::string cmd, int arg1) {
	BaseWriteCommand(cmd, std::to_string(arg1));
}

void StarGazerMessageProcessor::BaseReadCommand(std::string cmd) {
	BaseCommand(STAR_GAZER_MESSAGE_TYPES::READ, cmd);
}

void StarGazerMessageProcessor::CalcStop()
{
	BaseWriteCommand("CalcStop");
}


void StarGazerMessageProcessor::CalcStart()
{
	BaseWriteCommand("CalcStart");
}

void StarGazerMessageProcessor::SetMarkType(STAR_GAZER_LANDMARK_TYPES type)
{
	std::string cmd("MarkType");
	std::string arg1("");
	switch (type) {
	case STAR_GAZER_LANDMARK_TYPES::HLD1S:
		arg1 += "HLD1S";
		break;
	case STAR_GAZER_LANDMARK_TYPES::HLD1L:
		arg1 += "HLD1L";
		break;
	case STAR_GAZER_LANDMARK_TYPES::HLD2S:
		arg1 += "HLD2S";
		break;
	case STAR_GAZER_LANDMARK_TYPES::HLD2L:
		arg1 += "HLD2L";
		break;
	case STAR_GAZER_LANDMARK_TYPES::HLD3S:
		arg1 += "HLD3S";
		break;
	case STAR_GAZER_LANDMARK_TYPES::HLD3L:
		arg1 += "HLD3L";
		break;
	default:
		// Throw error
		throw std::runtime_error("unknown Marktype");
	}
	BaseWriteCommand(cmd, arg1);
}

void StarGazerMessageProcessor::GetVersion() 
{
	BaseReadCommand("Version");
}



void StarGazerMessageProcessor::HeightCalc() 
{
	BaseWriteCommand("HeightCalc");
}

void StarGazerMessageProcessor::SetMarkHeight(int height_mm)
{
	BaseWriteCommand("MarkHeight", height_mm);
}

void StarGazerMessageProcessor::SetEnd() 
{
	BaseWriteCommand("SetEnd");
}

void StarGazerMessageProcessor::PumpMessageProcessor() 
{
	std::chrono::duration<double> time_span = std::chrono::duration_cast<std::chrono::duration<double>>(m_highrezclk.now() - m_lastTxTime);

	if (m_lastTxMessage == m_lastAck)
	{
		std::cout << "Got Ack for " << m_lastAck << " took "  << time_span.count() << " seconds. " << std::endl;

		// Make sure we consume the ACK so that we don't get here again
		m_lastTxMessage = "";

		// Remove the message we have confirmed
		m_txMessageQueue.pop();

		// If there is another message to send, then send it.  Otherwise do nothing
		if (m_txMessageQueue.size() > 0) 
		{
			ROS_DEBUG_NAMED("StarGazerMessageProcessor", "Sending next message %s\n", m_txMessageQueue.front().c_str());
			SendRawCommand(m_txMessageQueue.front());
		}
		else {
			ROS_DEBUG_NAMED("StarGazerMessageProcessor", "Message Queue Empty\n");
		}
	}
	// If we have timed out - resend the message
	else if (m_lastTxMessage.size() > 0 && time_span.count() > STARTGAZER_TIMEOUT)
	{
		std::cout << "No Ack in " << time_span.count() << ", retransmitting " << m_txMessageQueue.front().c_str() << std::endl;

		ROS_DEBUG_NAMED("StarGazerMessageProcessor", "ACK timeout - retransmitting %s\n", m_txMessageQueue.front().c_str());

		SendRawCommand(m_txMessageQueue.front());
	}
}

//////////////////////////////////////////////////////////////////////////
// StarGazer Callbacks
//////////////////////////////////////////////////////////////////////////

void StarGazerMessageProcessor::RxMsgCallback(std::vector<char> msgBuffer)
{
	bool printMessage = false;

	STAR_GAZER_MESSAGE_TYPES command = (STAR_GAZER_MESSAGE_TYPES)msgBuffer[1];

	auto typeStart = msgBuffer.begin() + 2;
	auto typeEnd = std::find(typeStart, msgBuffer.end(), STARGAZER_SEPERATOR);

	std::string typeStr(typeStart, typeEnd);

	std::smatch regexMatch;
	std::string regexString = std::string(msgBuffer.begin(), msgBuffer.end());

	switch (command)
	{
	case STAR_GAZER_MESSAGE_TYPES::POSE:
	{
		if (std::regex_match(regexString, regexMatch, m_odometryRegex))
		{
			if (regexMatch.size() != 6) {
				ROS_DEBUG_NAMED("StarGazerMessageProcessor", "Odometry with only %i fields: %s\n",
					regexMatch.size(), std::string(msgBuffer.begin(), msgBuffer.end()).c_str());
			} 
			else if (m_odometryCallback)
			{
				try
				{
					int tagID = std::stoul(regexMatch[1]);

					float angle = std::stof(regexMatch[2]);

					float x = std::stof(regexMatch[3]);

					float y = std::stof(regexMatch[4]);

					float z = std::stof(regexMatch[5]);

					m_odometryCallback(tagID, x, y, z, angle);
				} 
				catch (const std::invalid_argument&) 
				{
					ROS_ERROR_NAMED("StarGazerMessageProcessor", "Odometry has invalid parameter\n");
				}
				catch (const std::out_of_range&)
				{
					ROS_ERROR_NAMED("StarGazerMessageProcessor", "Odometry has parameter out of range\n");
				}
			}
			else
			{
				ROS_ERROR_NAMED("StarGazerMessageProcessor", "Serial port data read but no callback specified!\n");
			}
		}
		else
		{
			ROS_DEBUG_NAMED("StarGazerMessageProcessor", "Odometry with bad format: %s\n",
				std::string(msgBuffer.begin(), msgBuffer.end()).c_str());
		}
	}
		break;
	case STAR_GAZER_MESSAGE_TYPES::MESSAGE:
		//std::cout << "Message " << typeStr << std::endl;
		ROS_DEBUG_NAMED("StarGazerMessageProcessor", "%s\n", typeStr.c_str());
		break;
	case STAR_GAZER_MESSAGE_TYPES::ACK:
		//std::cout << "Ack " << typeStr << std::endl;
		m_lastAck = typeStr;
		break;
	case STAR_GAZER_MESSAGE_TYPES::RETURN_VALUE:
		
		if (std::regex_match(regexString, regexMatch, m_messageRegex))
		{
			if (regexMatch.size() != 3) {
				ROS_DEBUG_NAMED("StarGazerMessageProcessor", "Return value with only %d fields: %s\n",
					regexMatch.size(), std::string(msgBuffer.begin(), msgBuffer.end()).c_str());
			}
			else if (m_readCallback)
			{
				std::string param("Not Implemented");
				m_readCallback(typeStr, regexMatch[2]);
			}
			else
			{
				ROS_ERROR_NAMED("StarGazerMessageProcessor", "Serial port data read but no callback specified!\n");
				printMessage = true;
			}
		}
		else
		{
			ROS_DEBUG_NAMED("StarGazerMessageProcessor", "Return value with bad format: %s\n",
				std::string(msgBuffer.begin(), msgBuffer.end()).c_str());
			printMessage = true;

		}


		
		break;
	default:
		std::cout << "Unknown command type " << (char)command << " ";
		printMessage = true;
		break;
	}

	if (printMessage)
	{
		std::cout << "Raw Message: \"";
		for (char value : msgBuffer)
		{
			std::cout << value;
		}
		std::cout << "\" " << std::endl;
	}
}

void StarGazerMessageProcessor::SetConnected(bool bIsConnected)
{
	/*
	std_msgs::Bool msg;
	msg.data = bIsConnected;

	m_ConnectedPublisher.publish( msg );

	m_VelocityPublisher.publish( geometry_msgs::Twist( ) );
	*/
}



//////////////////////////////////////////////////////////////////////////
// Helper Methods
//////////////////////////////////////////////////////////////////////////

void StarGazerMessageProcessor::WriteToSerialPort(char* pszData, std::size_t dwSize)
{
	if( !m_bControllerFault )
	{
		if( m_pIO->IsOpen( ) )
		{
			m_pIO->Write( std::vector<char>( pszData, pszData + dwSize ) );
		}
		else
		{
			ROS_ERROR_THROTTLE_NAMED( 60, "BrainStem", "Attempt to write to the brain stem, but the serial port is not open!" );
		}
	}
}

} /* namespace srs */
