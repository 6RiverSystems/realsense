/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <string>
#include <ros/ros.h>

using namespace std;

namespace srs {

class HardwareMessage
{
public:

	HardwareMessage(vector<char>& messageBuffer) :
		buffer_(messageBuffer),
		iter_()
	{
		iter_ = buffer_.begin();
	}

    virtual ~HardwareMessage() {}

    template <class T>
    bool checkBufferSize(uint32_t numObjects = 1, bool required = false)
    {
    	int32_t expected = sizeof(T) * numObjects;

    	int32_t actual = std::distance(iter_, buffer_.end());

    	bool enoughRoom = actual >= expected;

    	if (!enoughRoom && required)
    	{
        	ROS_ERROR_STREAM("Packet too small: expected=" << expected << ", actual=" << actual);
    	}

    	return enoughRoom;
    }

    template<typename T>
    T read()
    {
    	if (checkBufferSize<T>(1, true))
    	{
    		T value = *reinterpret_cast<T*>(&*iter_);

			iter_ += sizeof(value);

			return std::move(value);
    	}
    	else
    	{
    		throw std::runtime_error("Invalid hardware message (size of packet is too small)");
    	}
    }

    string readString()
    {
    	if (checkBufferSize<char>(2, true))
    	{
    		string value = string(reinterpret_cast<char*>(&*iter_));

    		iter_ += value.length() + 1;

    		return value;
    	}
    	else
    	{
  	      throw std::runtime_error("Invalid hardware message (size of packet is too small)");
    	}
    }

    template<typename T>
    void writeArray(T* value, int count)
    {
    	for (int i = 0; i < count; i++)
    	{
    		write<T>(value[i]);
    	}
    }

    template<typename T>
    void write(T value)
    {
        std::copy((uint8_t*) &value, ((uint8_t*) &value) + sizeof(T), std::back_inserter(buffer_));
    }

    void write(const string& value)
    {
        std::copy(value.c_str(), value.c_str()+value.length()+1, std::back_inserter(buffer_));
    }

private:

    vector<char>::iterator iter_;

    vector<char>& buffer_;
};

} // namespace srs
