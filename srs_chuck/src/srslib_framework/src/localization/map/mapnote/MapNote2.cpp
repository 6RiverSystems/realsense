#include <srslib_framework/localization/map/mapnote/MapNote2.hpp>

#include <srslib_framework/localization/map/mapnote/WarningSound.hpp>
#include <srslib_framework/localization/map/mapnote/MaxVelocity.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Constant declaration

const string MapNote2::NOTE_WARNING_SOUND = "warning_sound";
const string MapNote2::NOTE_MAX_VELOCITY = "max_velocity";

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
MapNote2* MapNote2::instanceOf(const MapNote2& originalNote)
{
    return nullptr;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
const MapNote2* MapNote2::instanceOf(const string& field)
{
    auto it = SYMBOL_2_NOTE.find(field);
    if (it != SYMBOL_2_NOTE.end())
    {
        return it->second;
    }

    return nullptr;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
MapNote2* MapNote2::instanceOf(const string& field, float value)
{
    MapNote2* note = nullptr;

    auto it = SYMBOL_2_NOTE.find(field);
    if (it == SYMBOL_2_NOTE.end())
    {
        if (field == NOTE_MAX_VELOCITY)
        {
            note = new MaxVelocity(value);
        }
    }

    return note;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods

//////////////////////////////////////////////////////////////////////////////////////////////////////
const unordered_map<string, const MapNote2*> MapNote2::SYMBOL_2_NOTE = {
    {"warning_sound", &WarningSound::WARNING_SOUND}
};

//////////////////////////////////////////////////////////////////////////////////////////////////////
unordered_map<int, string> MapNote2::NOTE_TYPE_2_STRING = {
    {MapNote2::NONE, "NONE"},
    {MapNote2::WARNING_SOUND, "WARNING_SOUND"},
    {MapNote2::MAX_VELOCITY, "MAX_VELOCITY"}
};

} // namespace srs
