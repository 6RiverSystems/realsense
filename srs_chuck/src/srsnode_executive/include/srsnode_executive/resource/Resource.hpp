/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

namespace srs {

class Resource
{
public:
    Resource(int initialRequest = 0) :
        requestCounter_(initialRequest),
        previousCounter_(initialRequest)
    {}

    virtual ~Resource()
    {}

    bool confirmed()
    {
        return requestCounter_;
    }

    void freeze()
    {
        if (requestCounter_ > 0)
        {
            requestCounter_ = 1;
        }
        previousCounter_ = requestCounter_;
    }

    bool hasChanged()
    {
        return (previousCounter_ > 0) != (requestCounter_ > 0);
    }

    void release()
    {
        requestCounter_ = requestCounter_ > 0 ? requestCounter_ - 1 : 0;
    }

    void request()
    {
        requestCounter_++;
    }

protected:
    int previousCounter_;
    int requestCounter_;
};

} // namespace srs
