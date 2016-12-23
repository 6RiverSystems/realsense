/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

using namespace std;

namespace srs {

template<typename TYPE, typename MESSAGE>
class PublisherInterface
{
public:

    virtual ~PublisherInterface() {}

    virtual std::string getTopic() const = 0;

    virtual void publish(TYPE data) = 0;

    virtual MESSAGE convertData(TYPE data) = 0;
};

} // namespace srs
