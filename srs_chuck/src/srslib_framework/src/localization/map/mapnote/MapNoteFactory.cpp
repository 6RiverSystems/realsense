#include <srslib_framework/localization/map/mapnote/MapNoteFactory.hpp>

#include <srslib_framework/localization/map/mapnote/NotePlaySound.hpp>
#include <srslib_framework/localization/map/mapnote/NoteQueue.hpp>
#include <srslib_framework/localization/map/mapnote/NoteSetMaxVelocity.hpp>
#include <srslib_framework/localization/map/mapnote/NoteStayOnTrack.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Constant declaration

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
MapNote::BaseMapNoteType MapNoteFactory::instanceOf(const string& field, const string& value)
{
    MapNote::BaseMapNoteType note = nullptr;

    if (field == NotePlaySound::TYPE)
    {
        note = shared_ptr<NotePlaySound>(new NotePlaySound(value));
    }
    else if (field == NoteSetMaxVelocity::TYPE)
    {
        note = shared_ptr<NoteSetMaxVelocity>(new NoteSetMaxVelocity(value));
    }
    else if (field == NoteQueue::TYPE)
    {
        note = shared_ptr<NoteQueue>(new NoteQueue(value));
    }
    else if (field == NoteStayOnTrack::TYPE)
    {
        note = shared_ptr<NoteStayOnTrack>(new NoteStayOnTrack());
    }

    return note;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods

} // namespace srs
