/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <string>
using namespace std;

#include <srslib_framework/Sound.h>

#include <srslib_framework/ros/channel/publisher/Publisher.hpp>
#include <srslib_framework/ros/message/SoundMessageFactory.hpp>

namespace srs {

class PublisherSound :
    public Publisher<const Sound&, srslib_framework::Sound>
{
public:
    PublisherSound(string topic,
        unsigned int buffer = 100,
        bool latched = false,
        string nameSpace = "~") :
            Publisher(topic, buffer, latched, nameSpace)
    {}

    srslib_framework::Sound convertData(const Sound& data)
    {
        return SoundMessageFactory::sound2Msg(data);
    }
};

} // namespace srs