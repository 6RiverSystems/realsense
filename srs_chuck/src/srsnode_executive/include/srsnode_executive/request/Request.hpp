/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

namespace srs {

class Request
{
public:
    Request(int initialRequest = 0) :
        requestCounter_(initialRequest),
        previousCounter_(initialRequest)
    {}

    virtual ~Request()
    {}

    bool confirmed()
    {
        if (requestCounter_ > 0)
        {
            requestCounter_ = 1;
        }

        previousCounter_ = requestCounter_;
        return requestCounter_;
    }

    bool execute()
    {
        if (hasChanged())
        {
            task();
            return true;
        }

        return false;
    }

    bool hasChanged()
    {
        return previousCounter_ != requestCounter_;
    }

    void release()
    {
        requestCounter_ = requestCounter_ > 0 ? requestCounter_ - 1 : 0;
    }

    void request()
    {
        requestCounter_++;
    }

    virtual void task() = 0;

protected:
    int previousCounter_;
    int requestCounter_;
};

} // namespace srs
