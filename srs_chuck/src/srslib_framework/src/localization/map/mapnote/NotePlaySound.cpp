#include <srslib_framework/localization/map/mapnote/NotePlaySound.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Constant declaration

const string NotePlaySound::TYPE = "play_sound";

const string NotePlaySound::SOUND_NONE = "none";
const string NotePlaySound::SOUND_WARNING = "warning";

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods

//////////////////////////////////////////////////////////////////////////////////////////////////////
unordered_map<string, int> NotePlaySound::STRING_2_SOUND_TYPE = {
    {NotePlaySound::SOUND_NONE, static_cast<int>(NotePlaySound::SoundType::NONE)},
    {NotePlaySound::SOUND_WARNING, static_cast<int>(NotePlaySound::SoundType::WARNING)}
};

//////////////////////////////////////////////////////////////////////////////////////////////////////
unordered_map<int, string> NotePlaySound::SOUND_TYPE_2_STRING = {
    {static_cast<int>(NotePlaySound::SoundType::NONE), NotePlaySound::SOUND_NONE},
    {static_cast<int>(NotePlaySound::SoundType::WARNING), NotePlaySound::SOUND_WARNING}
};

} // namespace srs
