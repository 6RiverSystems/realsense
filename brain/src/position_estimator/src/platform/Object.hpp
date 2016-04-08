/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef OBJECT_HPP_
#define OBJECT_HPP_

#include <string>
using namespace std;

namespace srs {

class Object
{
public:
    Object()
    {}

    ~Object()
    {}

    string toString()
    {
        char buffer[10];
        sprintf(buffer, "%p", this);

        return string(buffer);
    }
};

} // namespace srs

#endif // OBJECT_HPP_
