/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

namespace srs {

template <class SUBJECT>
class Observer
{
public:
    Observer()
    {}

    virtual ~Observer()
    {}

    virtual void notified(SUBJECT* subject) = 0;
};

} // namespace srs
