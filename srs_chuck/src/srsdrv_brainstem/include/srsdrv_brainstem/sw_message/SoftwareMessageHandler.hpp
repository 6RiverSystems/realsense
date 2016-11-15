/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <std_msgs/Bool.h>

namespace srs {

class BrainStemMessageProcessor;

class SoftwareMessageHandler
{
public:
    SoftwareMessageHandler(BrainStemMessageProcessor* owner) :
        owner_(owner)
    {}

    virtual ~SoftwareMessageHandler()
    {}

    BrainStemMessageProcessor* getOwner() const
    {
        return owner_;
    }

private:
    BrainStemMessageProcessor* owner_;
};

} // namespace srs
