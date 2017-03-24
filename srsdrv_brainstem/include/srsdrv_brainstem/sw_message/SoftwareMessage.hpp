/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <BrainStemMessages.hpp>
#include <sw_message/SoftwareMessage.hpp>

namespace srs {

class SoftwareMessage
{
public:
	SoftwareMessage(BrainStemMessageProcessorInterface* brainstemProcessor) :
		messageProcessor_(brainstemProcessor),
		valid_(false) {};

    virtual ~SoftwareMessage() {}

    virtual void attach() {};

    virtual void sync()
    {
    	if (valid_)
    	{
    		syncState();
    	}
    };

protected:

    virtual void syncState() {};

    void sendCommand(char* command, std::size_t size)
    {
    	messageProcessor_->sendCommand(command, size);

    	valid_ = true;
    }

    BrainStemMessageProcessorInterface* messageProcessor_;

private:

    bool valid_;
};

typedef std::shared_ptr<SoftwareMessage> SoftwareMessagePtr;

} // namespace srs
