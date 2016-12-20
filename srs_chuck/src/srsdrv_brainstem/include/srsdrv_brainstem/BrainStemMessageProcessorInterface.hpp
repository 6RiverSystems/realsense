/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <memory>

using namespace std;

namespace srs {

class BrainStemMessageProcessorInterface
{
public:

	virtual ~BrainStemMessageProcessorInterface() {};

    virtual void sendCommand(char* command, std::size_t size) = 0;

    virtual void sentPing() = 0;
};

} /* namespace srs */
