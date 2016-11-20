#include <srslib_framework/localization/map/MapNote.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Constant initialization

const MapNote MapNote::WARNING_BEEP = MapNote(true);

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
MapNote* MapNote::instanceOf(const MapNote& originalNote)
{
    MapNote* newNote = new MapNote();

    newNote->warning_beep_ = originalNote.warning_beep_;

    return newNote;
}

} // namespace srs
