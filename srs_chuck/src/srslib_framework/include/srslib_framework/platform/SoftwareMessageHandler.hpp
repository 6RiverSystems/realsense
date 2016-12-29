/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

namespace srs {

template<class OWNER>
class SoftwareMessageHandler
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

    virtual void attach() {};

private:
    OWNER* owner_;
};

} // namespace srs
