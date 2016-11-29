#include <srsnode_executive/sw_message/CommandLineHandler.hpp>

#include <algorithm>
#include <string>
using namespace std;

#include <boost/tokenizer.hpp>

#include <ros/ros.h>

#include <srsnode_executive/Executive.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
CommandLineHandler::CommandLineHandler(Executive* owner) :
    SoftwareMessageHandler<Executive>(owner),
    commandPause_(owner)
{
    tapCl_.attach(this);

    commandMap_["pause"] = &commandPause_;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void CommandLineHandler::notified(Subscriber<std_msgs::String>* subject)
{
    TapExecutiveCmd_Cl* tap = static_cast<TapExecutiveCmd_Cl*>(subject);

    string commandList = tap->pop();

    ROS_DEBUG_STREAM_NAMED("command_line_handler", "Executive received: " << commandList);
    processString(commandList);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods

////////////////////////////////////////////////////////////////////////////////////////////////////
void CommandLineHandler::processString(const string& message)
{
    string trimmedMessage;
    trimmedMessage.resize(message.length());
    transform(message.begin(), message.end(), trimmedMessage.begin(), ::tolower);

    boost::tokenizer<boost::char_separator<char>> commandListTokenizer(trimmedMessage,
        boost::char_separator<char>(";"));

    for (auto&& singleCommand: commandListTokenizer)
    {
        ROS_DEBUG_STREAM_NAMED("command_line_handler", "Parsing command: " << singleCommand);

        boost::tokenizer<boost::char_separator<char>> commandTokenizer(singleCommand,
            boost::char_separator<char>(" "));

        vector<string> commandTokens;
        for (auto&& token : commandTokenizer)
        {
            commandTokens.push_back(token);
        }

        if (!commandTokens.empty())
        {
            const std::string& command = commandTokens[Command::COMMAND];

            auto it = commandMap_.find(command);
            if (it != commandMap_.end())
            {
                if (commandTokens.size() >= it->second->getMinExpectedArguments() + 1)
                {
                    it->second->execute(commandTokens);
                }
                else
                {
                    ROS_ERROR_STREAM_NAMED("command_line_handler",
                        "Invalid number of expected arguments parsing [" << singleCommand << "]." <<
                        "Min expected: " << it->second->getMinExpectedArguments());
                }
            }
            else
            {
                ROS_ERROR_STREAM_NAMED("command_line_handler",
                    "Unknown command [" << command << "]");
            }
        }
    }
}

} // namespace srs
