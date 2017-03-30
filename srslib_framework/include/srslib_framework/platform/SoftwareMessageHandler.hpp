/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <srslib_framework/platform/observer/Observer.hpp>
#include <srslib_framework/ros/tap/subscriber/Subscriber.hpp>

namespace srs {

template<class OWNER, class MESSAGE_TYPE>
class SoftwareMessageHandler :
    public Observer<Subscriber<MESSAGE_TYPE>>
{
public:
    SoftwareMessageHandler(OWNER* owner) :
        owner_(owner)
    {}

    virtual ~SoftwareMessageHandler()
    {}

    OWNER* getOwner() const
    {
        return owner_;
    }

    virtual void attach()
    {}

    virtual void notified(Subscriber<MESSAGE_TYPE>* subject)
    {}

private:

    OWNER* owner_;
};

} // namespace srs
