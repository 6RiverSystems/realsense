#include <srslib_framework/localization/map/mapnote/MapNotes.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
bool MapNotes::add(BaseMapNoteType note)
{
    pair<MapBaseType::iterator, bool> result = MapBaseType::insert({
        static_cast<int>(note->getType()), note});

    return result.second;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
bool MapNotes::add(const MapNote::NoteTypeEnum nodeType, string value)
{
    BaseMapNoteType note = MapNote::instanceOf(nodeType, value);

    if (note)
    {
        pair<MapBaseType::iterator, bool> result = MapBaseType::insert({
            static_cast<int>(note->getType()), note});

        return result.second;
    }

    return false;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
bool MapNotes::add(const string& field, string value)
{
    BaseMapNoteType note = MapNote::instanceOf(field, value);

    if (note)
    {
        pair<MapBaseType::iterator, bool> result = MapBaseType::insert({
            static_cast<int>(note->getType()), note});

        return result.second;
    }

    return false;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
bool MapNotes::has(const MapNote::NoteTypeEnum nodeType) const
{
    return MapBaseType::find(nodeType) != MapBaseType::end();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods

} // namespace srs

