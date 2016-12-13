/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <memory>
#include <string>
#include <sstream>
#include <unordered_map>
using namespace std;

namespace srs {

class MapNote
{
public:
    using BaseMapNoteType = shared_ptr<MapNote>;

    MapNote()
    {}

    virtual ~MapNote()
    {}

    virtual bool equals(const MapNote& rhs) const
    {
        return getType() == rhs.getType();
    }

    virtual string getType() const = 0;
    virtual string getValue() const = 0;

    friend ostream& operator<<(ostream& stream, const MapNote& note)
    {
        stream << "{";
        note.toString(stream);
        return stream << "}";
    }

    virtual ostream& toString(ostream& stream) const
    {
        return stream << getType();
    }

private:
};

} // namespace srs
