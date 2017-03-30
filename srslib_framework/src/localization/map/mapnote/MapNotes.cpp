#include <srslib_framework/localization/map/mapnote/MapNotes.hpp>

#include <srslib_framework/localization/map/mapnote/MapNoteFactory.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
bool MapNotes::add(BaseMapNoteType note)
{
    pair<MapBaseType::iterator, bool> result = MapBaseType::insert({note->getType(), note});

    return result.second;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
bool MapNotes::add(const string& noteType, const string& value)
{
    BaseMapNoteType note = MapNoteFactory::instanceOf(noteType, value);

    if (note)
    {
        pair<MapBaseType::iterator, bool> result = MapBaseType::insert({note->getType(), note});

        return result.second;
    }

    return false;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
bool MapNotes::has(const string& nodeType) const
{
    return MapBaseType::find(nodeType) != MapBaseType::end();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods

} // namespace srs

