/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <memory>

namespace srs {

struct Template
{
    template<typename T>
    T* ptr(T& objectReference)
    {
        return &objectReference;
    }

    template<typename T>
    T* ptr(T* objectPointer)
    {
        return objectPointer;
    }

    template<typename T>
    T* ptr(shared_ptr<T>& objectSharedPointer)
    {
        return objectSharedPointer->get();
    }
};

} // namespace srs
