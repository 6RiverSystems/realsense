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

    virtual ~Object()
    {}

    virtual string toString()
    {
		std::ostringstream stringStream;
		stringStream << "Object: " << this;
        return stringStream.str();
    }
};

} // namespace srs

#endif // OBJECT_HPP_
