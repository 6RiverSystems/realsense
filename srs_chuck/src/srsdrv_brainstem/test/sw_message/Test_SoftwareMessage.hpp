/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <gtest/gtest.h>

#include <srslib_test/utils/Compare.hpp>

#include <srsdrv_brainstem/HardwareMessage.hpp>
#include <srsdrv_brainstem/BrainStemMessages.hpp>
#include "../../include/srsdrv_brainstem/BrainStemMessageProcessorInterface.hpp"

namespace srs {

class MockMessageProcessor :
	public BrainStemMessageProcessorInterface
{
public:

	virtual ~MockMessageProcessor() {};

    void sendCommand(char* command, std::size_t size)
    {
    	command_ = std::vector<char>(command, command + size);
    };

    void ping() { }

    std::vector<char> command_;
};

template<typename MESSAGE>
class Test_SoftwareMessage : public ::testing::Test
{
public:
	Test_SoftwareMessage() {};
	virtual ~Test_SoftwareMessage() {};

	virtual vector<char> createPacket(const MESSAGE& msg) = 0;

public:

	MockMessageProcessor messageProcessor_;
};

}
