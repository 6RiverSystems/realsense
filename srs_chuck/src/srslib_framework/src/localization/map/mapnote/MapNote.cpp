#include <srslib_framework/localization/map/mapnote/MapNote.hpp>

#include <memory>

#include <srslib_framework/localization/map/mapnote/Sound.hpp>
#include <srslib_framework/localization/map/mapnote/MaxVelocity.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Constant declaration

const string MapNote::NOTE_MAX_VELOCITY = "max_velocity";
const string MapNote::NOTE_NONE = "none";
const string MapNote::NOTE_SOUND = "sound";

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
MapNote::BaseMapNoteType MapNote::instanceOf(const NoteTypeEnum nodeType, string value)
{
    auto it = NOTE_TYPE_2_STRING.find(nodeType);
    if (it != NOTE_TYPE_2_STRING.end())
    {
        return instanceOf(it->second, value);
    }

    return nullptr;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
MapNote::BaseMapNoteType MapNote::instanceOf(const string& field, string value)
{
    shared_ptr<MapNote> note = nullptr;

    if (field == NOTE_MAX_VELOCITY)
    {
        note = shared_ptr<MaxVelocity>(new MaxVelocity(value));
    }
    else if (field == NOTE_SOUND)
    {
        note = shared_ptr<Sound>(new Sound(value));
    }

    return note;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods

//////////////////////////////////////////////////////////////////////////////////////////////////////
unordered_map<int, string> MapNote::NOTE_TYPE_2_STRING = {
    {MapNote::NONE, MapNote::NOTE_NONE},
    {MapNote::MAX_VELOCITY, MapNote::NOTE_MAX_VELOCITY},
    {MapNote::SOUND, MapNote::NOTE_SOUND}
};

} // namespace srs
