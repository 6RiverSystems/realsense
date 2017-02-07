/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <BrainStemMessages.hpp>

namespace srs {

template<class MESSAGE_TYPE>
class SendCommand
{
public:
    static void send(BrainStemMessageProcessorInterface* messageProcessor, MESSAGE_TYPE& message)
    {
        messageProcessor->sendCommand(reinterpret_cast<char*>(&message), sizeof(message));
    }
};

} // namespace srs
