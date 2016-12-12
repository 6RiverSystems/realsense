/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <gtest/gtest.h>

#include "../../include/srsdrv_brainstem/BrainStemMessages.hpp"

using namespace std;
using namespace srs;

#include <srslib_test/utils/Compare.hpp>

#include <hw_message/HardwareInfoHandler.hpp>
#include <srslib_framework/ros/channel/publisher/PublisherInterface.hpp>

using namespace srslib_framework;

template<typename TYPE, typename MESSAGE>
class MockPublisher : public PublisherInterface<TYPE, MESSAGE>
{
public:

	typedef std::function<MESSAGE(TYPE)> ConvertFn;

	MockPublisher(ConvertFn convert) :
		convert_(convert) {}

	virtual ~MockPublisher() {}

	virtual std::string getTopic() const { return ""; }

	virtual void publish(TYPE data)
	{
		data_ = convertData(data);
	}

	virtual MESSAGE convertData(TYPE data)
	{
		return convert_(data);
	}

	ConvertFn convert_;

	MESSAGE data_;
};

template<typename TYPE, typename MESSAGE>
class Test_HardwareMessage : public ::testing::Test
{
public:
	typedef std::function<MESSAGE(TYPE)> ConvertFn;

	Test_HardwareMessage(ConvertFn convert =
			[](TYPE data) {
		return data;
	}) : publisher_(convert) {};
	virtual ~Test_HardwareMessage() {};

	virtual vector<MESSAGE>& getTestMsgs() = 0;

	virtual vector<char> createPacket(const MESSAGE& msg, uint32_t version = 1) = 0;

public:

	MockPublisher<TYPE, MESSAGE> publisher_;
};
