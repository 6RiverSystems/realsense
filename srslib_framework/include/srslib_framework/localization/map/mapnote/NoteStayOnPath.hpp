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

class NoteStayOnPath : public MapNote
{
public:
    static const string TYPE;

    NoteStayOnPath() :
        MapNote()
    {}

    virtual ~NoteStayOnPath()
    {}

    std::string getType() const
    {
        return TYPE;
    }

    std::string getValue() const
    {
        return "1";
    }

    std::ostream& toString(std::ostream& stream) const
    {
        return stream << "type: " << getType();
    }

private:
};

} // namespace srs
