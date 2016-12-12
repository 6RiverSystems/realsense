#include <srslib_framework/localization/map/mapnote/MapNote.hpp>

#include <memory>

#include <srslib_framework/localization/map/mapnote/NotePlaySound.hpp>
#include <srslib_framework/localization/map/mapnote/NoteSetMaxVelocity.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Constant declaration

const string MapNote::NOTE_SET_MAX_VELOCITY = "set_max_velocity";
const string MapNote::NOTE_NONE = "none";
const string MapNote::NOTE_PLAY_SOUND = "play_sound";

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

    if (field == NOTE_SET_MAX_VELOCITY)
    {
        note = shared_ptr<NoteSetMaxVelocity>(new NoteSetMaxVelocity(value));
    }
    else if (field == NOTE_PLAY_SOUND)
    {
        note = shared_ptr<NotePlaySound>(new NotePlaySound(value));
    }

    return note;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods

////////////////////////////////////////////////////////////////////////////////////////////////////
unordered_map<int, string> MapNote::NOTE_TYPE_2_STRING = {
    {MapNote::NONE, MapNote::NOTE_NONE},
    {MapNote::SET_MAX_VELOCITY, MapNote::NOTE_SET_MAX_VELOCITY},
    {MapNote::PLAY_SOUND, MapNote::NOTE_PLAY_SOUND}
};

} // namespace srs
