#include <srslib_framework/localization/MapNote.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Constant initialization

const MapNote MapNote::DISABLE_OD = MapNote(false, false, false, false, false, 0);
const MapNote MapNote::ENABLE_OD = MapNote(true, false, false, false, false, 0);
const MapNote MapNote::GO_SLOW = MapNote(false, true, false, false, false, 0);
const MapNote MapNote::NO_ROTATIONS = MapNote(false, false, true, false, false, 0);
const MapNote MapNote::STATIC_OBSTACLE = MapNote(false, false, false, true, false, 0);
const MapNote MapNote::PREFERRED_0 = MapNote(false, false, false, false, true, 0);
const MapNote MapNote::PREFERRED_90 = MapNote(false, false, false, false, true, 90);
const MapNote MapNote::PREFERRED_180 = MapNote(false, false, false, false, true, 180);
const MapNote MapNote::PREFERRED_270 = MapNote(false, false, false, false, true, 270);

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
void MapNote::add(int8_t flags)
{
    reset();

    goSlow_ = flags & FLAG_GO_SLOW;
    noRotations_ = flags & FLAG_NO_ROTATIONS;
    od_ = flags & FLAG_OD;
    staticObstacle_ = flags & FLAG_STATIC_OBSTACLE;

    if (flags & FLAG_PREFERRED_2)
    {
        preferred_ = true;
        preferredAngle_ = MapNote::getAngle(flags);
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
int8_t MapNote::getFlags()
{
    int8_t notes = 0;

    notes |= od_ ? FLAG_OD : 0;
    notes |= noRotations_ ? FLAG_NO_ROTATIONS : 0;
    notes |= goSlow_ ? FLAG_GO_SLOW : 0;
    notes |= staticObstacle_ ? FLAG_STATIC_OBSTACLE : 0;

    if (preferred_)
    {
        notes |= FLAG_PREFERRED_2;

        switch (preferredAngle_)
        {
            case 0:
                notes |= FLAG_PREFERRED_000;
                break;

            case 90:
                notes |= FLAG_PREFERRED_090;
                break;

            case 180:
                notes |= FLAG_PREFERRED_180;
                break;

            case 270:
                notes |= FLAG_PREFERRED_270;
                break;
        }
    }

    return notes;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
MapNote* MapNote::instanceOf(int8_t flags)
{
    MapNote* note = new MapNote();
    note->reset();

    if (flags & MapNote::FLAG_GO_SLOW)
    {
        note->join(MapNote::GO_SLOW);
    }

    if (flags & MapNote::FLAG_NO_ROTATIONS)
    {
        note->join(MapNote::NO_ROTATIONS);
    }

    if (flags & MapNote::FLAG_OD)
    {
        note->join(MapNote::ENABLE_OD);
    }
    else
    {
        note->join(MapNote::DISABLE_OD);
    }

    if (flags & MapNote::FLAG_STATIC_OBSTACLE)
    {
        note->join(MapNote::STATIC_OBSTACLE);
    }

    if (flags & MapNote::FLAG_PREFERRED_2)
    {
        switch (MapNote::getAngleCode(flags))
        {
            case 0:
                note->join(MapNote::PREFERRED_0);
                break;
            case 1:
                note->join(MapNote::PREFERRED_90);
                break;
            case 2:
                note->join(MapNote::PREFERRED_180);
                break;
            case 3:
                note->join(MapNote::PREFERRED_270);
                break;
        }
    }

    return note;
}

} // namespace srs
