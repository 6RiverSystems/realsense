#include <srslib_framework/localization/MapNote.hpp>

namespace srs {

const MapNote MapNote::DISABLE_OD = MapNote(false, false, false, false);
const MapNote MapNote::ENABLE_OD = MapNote(true, false, false, false);
const MapNote MapNote::GO_SLOW = MapNote(false, true, false, false);
const MapNote MapNote::NO_ROTATIONS = MapNote(false, false, true, false);
const MapNote MapNote::STATIC_OBSTACLE = MapNote(false, false, false, true);

}
