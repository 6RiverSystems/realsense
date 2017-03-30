/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <vector>
using namespace std;

#include <srslib_framework/platform/observer/Observer.hpp>

namespace srs {

template <class SUBJECT>
class Subject
{
public:
    Subject()
    {}

    virtual ~Subject()
    {}

    void attach(Observer<SUBJECT>* observer)
    {
        observers_.push_back(observer);
    }

    void notify()
    {
        for (auto observer : observers_)
        {
            observer->notified(static_cast<SUBJECT*>(this));
        }
    }

private:
    vector<Observer<SUBJECT>*> observers_;
};

} // namespace srs
