/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <string>
#include <sstream>

#include <srslib_framework/localization/map/mapnote/MapNote.hpp>

namespace srs {

class NoteQueue : public MapNote
{
public:
    static const string TYPE;

    NoteQueue(string name) :
        MapNote(),
        name_(name)
    {}

    virtual ~NoteQueue()
    {}

    std::string getType() const
    {
        return TYPE;
    }

    std::string getValue() const
    {
        return name_;
    }

    std::ostream& toString(std::ostream& stream) const
    {
        return stream << "type: " << getType() << ", name: " << getValue();
    }

    std::string getName() const
    {
        return name_;
    }

private:
    std::string name_;
};

} // namespace srs
